#!/usr/bin/env python3
"""
Deep Koopman FDI Node for Exoskeleton Digital Twin — Rev.2
===========================================================

Data-driven Fault Detection and Isolation using the Koopman operator
with learned lifting functions and a bank of Luenberger observers in
the lifted linear space.

System modeling
---------------
The reduced plant equation is:

    M_eff · θ̈ = τ_m + τ_pass + τ_ext - proj - τ_fric - τ_damp

From the Koopman perspective, this is a SIMO system with:

  STATE  x = [θ, θ̇]           (2D)  — measured via /joint_states
  INPUT  u = [τ_m, τ_ext]      (2D)  — both are external inputs to the plant
  OUTPUT y = [θ, θ̇]           (2D)  — same as state (fully measured)

Note: τ_ext is the force of human-machine interaction projected onto θ.
It enters the plant equation as an INPUT (not a state), because it is
an external force applied to the mechanism, measured by a force sensor,
and published on /exo_dynamics/tau_ext_theta.

Architecture overview
---------------------
PHASE 1 — DATA COLLECTION + TRAINING (offline, runs once)
  1. Collects healthy input-output trajectories from the running DT.
  2. Trains a neural network (encoder) to learn lifting functions φ(x)
     that map x ∈ R² into a higher-dimensional space z = [x_norm; φ(x)]
     where the dynamics are approximately linear:
         z_{k+1} ≈ A·z_k + B·u_k
  3. Identifies A, B via regularized least-squares.
  4. Builds a bank of Luenberger observers in the lifted space,
     each structured for isolation of a specific fault channel.

PHASE 2 — ONLINE FDI (continuous, 200 Hz)
  1. Lifts the measured state into Koopman space.
  2. Each observer predicts the next lifted state and computes a residual.
  3. Structured residual evaluation detects and isolates faults.

Fault channels (matching the fault_injector)
---------------------------------------------
  CH 0 — /exo_dynamics/tau_ext_theta  → force sensor fault (bias, drift)
  CH 1 — /trajectory_ref             → outer-loop fault (admittance controller)
  CH 2 — /torque                     → inner-loop fault (actuator torque)
  CH 3 — /joint_states               → encoder fault (position/velocity)

Subscribed topics
-----------------
  /joint_states                    sensor_msgs/JointState
  /exo_dynamics/tau_ext_theta      std_msgs/Float64
  /torque                          std_msgs/Float64
  /trajectory_ref                  std_msgs/Float64MultiArray
  /exo_dynamics/ff_terms           std_msgs/Float64MultiArray

Published topics
----------------
  /koopman_fdi/residuals           std_msgs/Float64MultiArray
  /koopman_fdi/fault_detected      std_msgs/Float64MultiArray
  /koopman_fdi/debug               std_msgs/Float64MultiArray
  /koopman_fdi/status              std_msgs/String

ROS2 Parameters
---------------
  joint_name            (str)    rev_crank
  publish_rate          (float)  200.0

  # Training
  training_duration     (float)  seconds of healthy data to collect (30.0)
  lifting_dim           (int)    extra lifted dims (16)
  training_epochs       (int)    NN training epochs (200)
  learning_rate         (float)  Adam LR (1e-3)

  # Observer bank
  observer_poles        (float)  pole magnitude in lifted space (0.85)

  # Detection
  detection_threshold   (float)  residual norm threshold (0.5)
  isolation_ratio       (float)  ratio for structured isolation (2.0)
  persistence_count     (int)    consecutive detections before alarm (10)

  # Adaptive thresholds
  adaptive_threshold    (bool)   use rolling stats (True)
  adaptive_sigma        (float)  number of sigma (5.0)
  adaptive_window       (int)    rolling window samples (1000)

  # Model persistence
  model_path            (str)    path to save/load model
  load_model            (bool)   skip training, load from file (False)
"""

import math
import os
from collections import deque
from enum import IntEnum
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String
from sensor_msgs.msg import JointState


# ═══════════════════════════════════════════════════════════════
# Neural Network Encoder — Pure NumPy (no PyTorch dependency)
# ═══════════════════════════════════════════════════════════════

class KoopmanEncoder:
    """
    Feedforward network that learns lifting functions φ(x).

    Architecture:  x ∈ R^n_x  →  [64] → [64] → [n_lift]  with tanh.
    Full lifted state:  z = [x_norm; φ(x)]  ∈ R^{n_x + n_lift}.

    Training minimizes the Koopman prediction loss:
        L = Σ_k || z_{k+1} - A·z_k - B·u_k ||²
    where A, B are re-estimated at each epoch via least-squares.
    """

    def __init__(self, n_x: int, n_u: int, n_lift: int, seed: int = 42):
        self.n_x = n_x
        self.n_u = n_u
        self.n_lift = n_lift
        self.n_z = n_x + n_lift

        rng = np.random.default_rng(seed)
        h1, h2 = 64, 64

        # Xavier initialization
        self.W1 = rng.normal(0, np.sqrt(2.0 / n_x), (n_x, h1))
        self.b1 = np.zeros(h1)
        self.W2 = rng.normal(0, np.sqrt(2.0 / h1), (h1, h2))
        self.b2 = np.zeros(h2)
        self.W3 = rng.normal(0, np.sqrt(2.0 / h2), (h2, n_lift))
        self.b3 = np.zeros(n_lift)

        # Koopman matrices
        self.A = np.eye(self.n_z)
        self.B = np.zeros((self.n_z, n_u))

        # Normalization
        self.x_mean = np.zeros(n_x)
        self.x_std = np.ones(n_x)
        self.u_mean = np.zeros(n_u)
        self.u_std = np.ones(n_u)

    def _forward_raw(self, x_norm: np.ndarray):
        """Forward pass, returns (phi, cache_for_backprop)."""
        a1 = x_norm @ self.W1 + self.b1
        h1 = np.tanh(a1)
        a2 = h1 @ self.W2 + self.b2
        h2 = np.tanh(a2)
        a3 = h2 @ self.W3 + self.b3
        phi = np.tanh(a3)
        return phi, (x_norm, a1, h1, a2, h2, a3)

    def lift(self, x: np.ndarray) -> np.ndarray:
        """Lift a single state x ∈ R^n_x → z ∈ R^n_z."""
        x_norm = (x - self.x_mean) / (self.x_std + 1e-8)
        phi, _ = self._forward_raw(x_norm)
        return np.concatenate([x_norm, phi])

    def normalize_u(self, u: np.ndarray) -> np.ndarray:
        """Normalize input vector."""
        return (u - self.u_mean) / (self.u_std + 1e-8)

    def lift_batch(self, X: np.ndarray) -> np.ndarray:
        """Lift batch X ∈ R^{N x n_x} → Z ∈ R^{N x n_z}."""
        X_norm = (X - self.x_mean) / (self.x_std + 1e-8)
        phi, _ = self._forward_raw(X_norm)
        return np.hstack([X_norm, phi])

    def _backprop(self, dL_dphi, cache):
        """Backprop through encoder."""
        x_norm, a1, h1, a2, h2, a3 = cache
        phi = np.tanh(a3)

        da3 = dL_dphi * (1.0 - phi ** 2)
        dW3 = h2.T @ da3
        db3 = da3.sum(axis=0)

        dh2 = da3 @ self.W3.T
        da2 = dh2 * (1.0 - h2 ** 2)
        dW2 = h1.T @ da2
        db2 = da2.sum(axis=0)

        dh1 = da2 @ self.W2.T
        da1 = dh1 * (1.0 - h1 ** 2)
        dW1 = x_norm.T @ da1
        db1 = da1.sum(axis=0)

        return dW1, db1, dW2, db2, dW3, db3

    def train(self, X_seq: np.ndarray, U_seq: np.ndarray,
              epochs: int = 200, lr: float = 1e-3, logger=None):
        """
        Train encoder + identify Koopman matrices A, B.

        X_seq: (N, n_x) — state trajectory  [θ, θ̇]
        U_seq: (N, n_u) — input trajectory  [τ_m, τ_ext]
        """
        N = X_seq.shape[0]

        # Normalization stats
        self.x_mean = X_seq.mean(axis=0)
        self.x_std = X_seq.std(axis=0)
        self.x_std[self.x_std < 1e-6] = 1.0
        self.u_mean = U_seq.mean(axis=0)
        self.u_std = U_seq.std(axis=0)
        self.u_std[self.u_std < 1e-6] = 1.0

        U_norm = (U_seq - self.u_mean) / (self.u_std + 1e-8)

        # Adam optimizer state
        params = [self.W1, self.b1, self.W2, self.b2, self.W3, self.b3]
        m_adam = [np.zeros_like(p) for p in params]
        v_adam = [np.zeros_like(p) for p in params]
        beta1, beta2, eps_adam = 0.9, 0.999, 1e-8

        best_loss = float('inf')
        patience_counter = 0

        for epoch in range(epochs):
            # Lift all states
            Z = self.lift_batch(X_seq)

            # Least-squares identification: Z[k+1] ≈ A·Z[k] + B·U[k]
            Z_curr = Z[:-1]
            Z_next = Z[1:]
            U_curr = U_norm[:-1]

            Phi = np.hstack([Z_curr, U_curr])
            lam = 1e-4
            AB = np.linalg.solve(
                Phi.T @ Phi + lam * np.eye(Phi.shape[1]),
                Phi.T @ Z_next
            )
            self.A = AB[:self.n_z].T
            self.B = AB[self.n_z:].T

            # Prediction error
            Z_pred = Z_curr @ self.A.T + U_curr @ self.B.T
            error = Z_next - Z_pred
            loss = np.mean(error ** 2)

            # Gradient of loss w.r.t. lifted states
            scale = 2.0 / ((N - 1) * self.n_z)
            dL_dZ_next = error * scale
            dL_dZ_curr = -error @ self.A * scale

            # Backprop through encoder (phi part only, first n_x cols are passthrough)
            dL_dphi_next = dL_dZ_next[:, self.n_x:]
            dL_dphi_curr = dL_dZ_curr[:, self.n_x:]

            X_norm_next = (X_seq[1:] - self.x_mean) / (self.x_std + 1e-8)
            _, cache_next = self._forward_raw(X_norm_next)
            grads_next = self._backprop(dL_dphi_next, cache_next)

            X_norm_curr = (X_seq[:-1] - self.x_mean) / (self.x_std + 1e-8)
            _, cache_curr = self._forward_raw(X_norm_curr)
            grads_curr = self._backprop(dL_dphi_curr, cache_curr)

            grads = [gn + gc for gn, gc in zip(grads_next, grads_curr)]

            # Adam update
            for i, (p, g) in enumerate(zip(params, grads)):
                m_adam[i] = beta1 * m_adam[i] + (1 - beta1) * g
                v_adam[i] = beta2 * v_adam[i] + (1 - beta2) * g ** 2
                m_hat = m_adam[i] / (1 - beta1 ** (epoch + 1))
                v_hat = v_adam[i] / (1 - beta2 ** (epoch + 1))
                p -= lr * m_hat / (np.sqrt(v_hat) + eps_adam)

            self.W1, self.b1 = params[0], params[1]
            self.W2, self.b2 = params[2], params[3]
            self.W3, self.b3 = params[4], params[5]

            if epoch % 20 == 0 and logger:
                logger.info(f"  Koopman training epoch {epoch}/{epochs}, "
                            f"loss={loss:.6f}")

            if loss < best_loss - 1e-7:
                best_loss = loss
                patience_counter = 0
            else:
                patience_counter += 1
            if patience_counter > 30:
                if logger:
                    logger.info(f"  Early stopping at epoch {epoch}, "
                                f"loss={loss:.6f}")
                break

        # Final least-squares fit
        Z = self.lift_batch(X_seq)
        Phi = np.hstack([Z[:-1], U_norm[:-1]])
        AB = np.linalg.solve(
            Phi.T @ Phi + lam * np.eye(Phi.shape[1]),
            Phi.T @ Z[1:]
        )
        self.A = AB[:self.n_z].T
        self.B = AB[self.n_z:].T

        final_pred = Z[:-1] @ self.A.T + U_norm[:-1] @ self.B.T
        final_loss = np.mean((Z[1:] - final_pred) ** 2)
        if logger:
            logger.info(f"  Training complete. Final loss={final_loss:.6f}")
            eigs = np.abs(np.linalg.eigvals(self.A))
            logger.info(f"  A spectral radius = {np.max(eigs):.4f}")

    def save(self, path: str):
        np.savez(path,
                 W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2,
                 W3=self.W3, b3=self.b3,
                 A=self.A, B=self.B,
                 x_mean=self.x_mean, x_std=self.x_std,
                 u_mean=self.u_mean, u_std=self.u_std,
                 n_x=self.n_x, n_u=self.n_u, n_lift=self.n_lift)

    @classmethod
    def load(cls, path: str):
        data = np.load(path)
        enc = cls(int(data['n_x']), int(data['n_u']), int(data['n_lift']))
        enc.W1, enc.b1 = data['W1'], data['b1']
        enc.W2, enc.b2 = data['W2'], data['b2']
        enc.W3, enc.b3 = data['W3'], data['b3']
        enc.A, enc.B = data['A'], data['B']
        enc.x_mean, enc.x_std = data['x_mean'], data['x_std']
        enc.u_mean, enc.u_std = data['u_mean'], data['u_std']
        return enc


# ═══════════════════════════════════════════════════════════════
# Luenberger Observer in Koopman (lifted) space
# ═══════════════════════════════════════════════════════════════

class KoopmanObserver:
    """
    Luenberger observer in lifted linear space.

    z_hat_{k+1} = A·z_hat_k + B·u_k + L·(C·z_meas_k - C·z_hat_k)

    The gain L is designed for desired convergence rate via pole_mag.
    """

    def __init__(self, A: np.ndarray, B: np.ndarray, C: np.ndarray,
                 pole_mag: float = 0.85):
        self.n_z = A.shape[0]
        self.n_u = B.shape[1]
        self.n_y = C.shape[0]
        self.A = A.copy()
        self.B = B.copy()
        self.C = C.copy()

        # Observer gain: L ≈ (1 - pole_mag) · A · C^T · (C·C^T)^{-1}
        CCt = C @ C.T
        if self.n_y > 0 and np.linalg.det(CCt) > 1e-12:
            CCt_inv = np.linalg.inv(CCt)
            self.L = (1.0 - pole_mag) * (A @ C.T @ CCt_inv)
        else:
            self.L = np.zeros((self.n_z, max(self.n_y, 1)))

        self.z_hat = np.zeros(self.n_z)
        self.residual = np.zeros(self.n_y)
        self.initialized = False

    def reset(self, z0: np.ndarray):
        self.z_hat = z0.copy()
        self.initialized = True

    def step(self, z_meas: np.ndarray, u_norm: np.ndarray) -> np.ndarray:
        """One observer step. Returns the innovation (residual)."""
        if not self.initialized:
            self.z_hat = z_meas.copy()
            self.initialized = True
            self.residual = np.zeros(self.n_y)
            return self.residual

        # Innovation
        y_meas = self.C @ z_meas
        y_hat = self.C @ self.z_hat
        self.residual = y_meas - y_hat

        # State update
        self.z_hat = (self.A @ self.z_hat
                      + self.B @ u_norm
                      + self.L @ self.residual)

        return self.residual


# ═══════════════════════════════════════════════════════════════
# FDI State Machine
# ═══════════════════════════════════════════════════════════════

class FDIState(IntEnum):
    WAITING_FOR_DATA = 0
    COLLECTING = 1
    TRAINING = 2
    MONITORING = 3
    ERROR = 4


# ═══════════════════════════════════════════════════════════════
# Main ROS2 Node
# ═══════════════════════════════════════════════════════════════

class KoopmanFDINode(Node):

    # Fault channel mapping (matches fault_injector)
    FAULT_CHANNELS = {
        'tau_ext_fault':   0,   # CH0: /exo_dynamics/tau_ext_theta
        'traj_ref_fault':  1,   # CH1: /trajectory_ref
        'torque_fault':    2,   # CH2: /torque
        'encoder_fault':   3,   # CH3: /joint_states
    }

    def __init__(self):
        super().__init__('koopman_fdi_node')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('joint_name', 'rev_crank')
        self.declare_parameter('publish_rate', 200.0)

        self.declare_parameter('training_duration', 30.0)
        self.declare_parameter('lifting_dim', 16)
        self.declare_parameter('training_epochs', 200)
        self.declare_parameter('learning_rate', 1e-3)

        self.declare_parameter('observer_poles', 0.85)

        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('isolation_ratio', 2.0)
        self.declare_parameter('persistence_count', 10)

        self.declare_parameter('adaptive_threshold', True)
        self.declare_parameter('adaptive_sigma', 5.0)
        self.declare_parameter('adaptive_window', 1000)

        self.declare_parameter('model_path',
                               str(Path.home() / 'koopman_fdi_model.npz'))
        self.declare_parameter('load_model', False)

        # Read params
        self.joint_name = self.get_parameter('joint_name').value
        rate = self.get_parameter('publish_rate').value
        self.dt_nom = 1.0 / rate

        self.training_duration = self.get_parameter('training_duration').value
        self.n_lift = self.get_parameter('lifting_dim').value
        self.training_epochs = self.get_parameter('training_epochs').value
        self.lr = self.get_parameter('learning_rate').value

        self.pole_mag = self.get_parameter('observer_poles').value

        self.det_threshold = self.get_parameter('detection_threshold').value
        self.iso_ratio = self.get_parameter('isolation_ratio').value
        self.persist_count = self.get_parameter('persistence_count').value

        self.adaptive_thresh = self.get_parameter('adaptive_threshold').value
        self.adaptive_sigma = self.get_parameter('adaptive_sigma').value
        self.adaptive_window = self.get_parameter('adaptive_window').value

        self.model_path = self.get_parameter('model_path').value
        self.load_model_flag = self.get_parameter('load_model').value

        # ── System dimensions ────────────────────────────────────
        #   x = [θ, θ̇]        → n_x = 2  (state / output)
        #   u = [τ_m, τ_ext]   → n_u = 2  (inputs to the plant)
        self.n_x = 2
        self.n_u = 2
        self.n_z = self.n_x + self.n_lift

        # ── Internal state ───────────────────────────────────────
        self.state = FDIState.WAITING_FOR_DATA

        # Measured signals (updated by callbacks)
        self.theta = 0.0
        self.theta_dot = 0.0
        self.tau_ext = 0.0
        self.tau_m = 0.0
        self.theta_ref = 0.0
        self.theta_dot_ref = 0.0
        self.theta_ddot_ref = 0.0
        self.M_eff = 1.0

        self.js_received = False
        self.tau_ext_received = False
        self.tau_m_received = False
        self.traj_ref_received = False

        # Training buffers
        self.train_X = []   # [θ, θ̇]
        self.train_U = []   # [τ_m, τ_ext]
        self.collect_start_time = None

        # Koopman model
        self.encoder = None

        # Observer bank:
        # - healthy:        uses all outputs → sensitive to ALL faults
        # - tau_ext_fault:   excludes τ_ext from input → insensitive to CH0
        # - traj_ref_fault:  separate detection via trajectory tracking error
        # - torque_fault:    excludes τ_m from input → insensitive to CH2
        # - encoder_fault:   excludes [θ,θ̇] from output → insensitive to CH3
        self.observer_names = [
            'healthy',
            'tau_ext_fault',
            'traj_ref_fault',
            'torque_fault',
            'encoder_fault',
        ]
        self.observers = {}

        # Residual statistics
        self.res_history = {}

        # Persistence counters
        self.persist_counters = {n: 0 for n in self.observer_names}

        # Fault declaration
        self.fault_detected = False
        self.fault_channel = -1

        # Trajectory tracking error for CH1 detection
        self.traj_error_history = deque(maxlen=self.adaptive_window)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(
            Float64, '/exo_dynamics/tau_ext_theta', self._tau_ext_cb, 10)
        self.create_subscription(
            Float64, '/torque', self._tau_m_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/trajectory_ref', self._traj_ref_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/exo_dynamics/ff_terms', self._ff_cb, 10)

        # ── Publishers ───────────────────────────────────────────
        self.pub_residuals = self.create_publisher(
            Float64MultiArray, '/koopman_fdi/residuals', 10)
        self.pub_fault = self.create_publisher(
            Float64MultiArray, '/koopman_fdi/fault_detected', 10)
        self.pub_debug = self.create_publisher(
            Float64MultiArray, '/koopman_fdi/debug', 10)
        self.pub_status = self.create_publisher(
            String, '/koopman_fdi/status', 10)

        # ── Main timer ───────────────────────────────────────────
        self.timer = self.create_timer(self.dt_nom, self._main_loop)

        # Try loading saved model
        if self.load_model_flag and os.path.exists(self.model_path):
            try:
                self.encoder = KoopmanEncoder.load(self.model_path)
                self._build_observer_bank()
                self.state = FDIState.MONITORING
                self.get_logger().info(
                    f"Loaded Koopman model from {self.model_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load model: {e}")
                self.state = FDIState.WAITING_FOR_DATA

        self.get_logger().info(
            f"KoopmanFDI Rev.2 | x=[θ,θ̇] (n_x={self.n_x}) | "
            f"u=[τ_m,τ_ext] (n_u={self.n_u}) | "
            f"lift_dim={self.n_lift} | n_z={self.n_z} | "
            f"fault channels: 0,1,2,3")

    # ═══════════════════════════════════════════════════════════
    # Callbacks
    # ═══════════════════════════════════════════════════════════

    def _js_cb(self, msg: JointState):
        try:
            idx = list(msg.name).index(self.joint_name)
        except ValueError:
            return
        if idx < len(msg.position):
            self.theta = float(msg.position[idx])
        if idx < len(msg.velocity):
            self.theta_dot = float(msg.velocity[idx])
        self.js_received = True

    def _tau_ext_cb(self, msg: Float64):
        self.tau_ext = float(msg.data)
        self.tau_ext_received = True

    def _tau_m_cb(self, msg: Float64):
        self.tau_m = float(msg.data)
        self.tau_m_received = True

    def _traj_ref_cb(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) > 0:
            self.theta_ref = float(d[0])
        if len(d) > 1:
            self.theta_dot_ref = float(d[1])
        if len(d) > 2:
            self.theta_ddot_ref = float(d[2])
        self.traj_ref_received = True

    def _ff_cb(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) > 0:
            val = float(d[0])
            self.M_eff = val if val > 1e-6 else 1.0

    # ═══════════════════════════════════════════════════════════
    # State machine
    # ═══════════════════════════════════════════════════════════

    def _main_loop(self):
        status_msg = String()
        status_msg.data = FDIState(self.state).name
        self.pub_status.publish(status_msg)

        if self.state == FDIState.WAITING_FOR_DATA:
            self._state_waiting()
        elif self.state == FDIState.COLLECTING:
            self._state_collecting()
        elif self.state == FDIState.TRAINING:
            self._state_training()
        elif self.state == FDIState.MONITORING:
            self._state_monitoring()

    def _state_waiting(self):
        if (self.js_received and self.tau_ext_received
                and self.tau_m_received):
            self.state = FDIState.COLLECTING
            self.collect_start_time = self.get_clock().now()
            self.get_logger().info(
                f"All topics received. Collecting {self.training_duration:.0f}s "
                f"of healthy data... (ensure fault_injector is INACTIVE)")

    def _state_collecting(self):
        x = np.array([self.theta, self.theta_dot])
        u = np.array([self.tau_m, self.tau_ext])
        self.train_X.append(x)
        self.train_U.append(u)

        elapsed = (self.get_clock().now() -
                   self.collect_start_time).nanoseconds * 1e-9
        if elapsed >= self.training_duration:
            self.get_logger().info(
                f"Collected {len(self.train_X)} samples. Training...")
            self.state = FDIState.TRAINING

    def _state_training(self):
        X = np.array(self.train_X)
        U = np.array(self.train_U)

        self.get_logger().info(
            f"Training data: X={X.shape} (state), U={U.shape} (input)")

        self.encoder = KoopmanEncoder(self.n_x, self.n_u, self.n_lift)
        self.encoder.train(X, U,
                           epochs=self.training_epochs,
                           lr=self.lr,
                           logger=self.get_logger())

        self._build_observer_bank()

        try:
            self.encoder.save(self.model_path)
            self.get_logger().info(f"Model saved to {self.model_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not save model: {e}")

        self.train_X.clear()
        self.train_U.clear()

        self.state = FDIState.MONITORING
        self.get_logger().info("Training complete — MONITORING active")

    # ═══════════════════════════════════════════════════════════
    # Observer bank construction
    # ═══════════════════════════════════════════════════════════

    def _build_observer_bank(self):
        """
        Build structured observer bank for fault isolation.

        The isolation principle:
        Each observer is designed to be INSENSITIVE to one specific
        fault channel. When a fault occurs, the insensitive observer
        will have small residuals while all others show large residuals.

        For input faults (CH0: τ_ext, CH2: τ_m):
          → The observer uses a MODIFIED B matrix that zeros out the
            column corresponding to the faulty input. This way, the
            faulty input does not affect the prediction, making the
            observer insensitive to that fault.

        For output/sensor fault (CH3: encoder):
          → The observer uses a REDUCED C matrix that excludes the
            corrupted measurements from the innovation.

        For reference fault (CH1: trajectory_ref):
          → Not directly observable in the Koopman model (it affects
            the closed-loop behavior indirectly through τ_m).
            Detected via trajectory tracking error monitoring.
        """
        A = self.encoder.A
        B = self.encoder.B

        # Full output matrix: observe the first n_x states of z
        # z = [θ_n, θ̇_n, φ_1, ..., φ_n_lift]
        C_full = np.zeros((self.n_x, self.n_z))
        C_full[:self.n_x, :self.n_x] = np.eye(self.n_x)

        # ── Observer 0 (healthy): full C, full B ──
        # Sensitive to ALL faults
        self.observers['healthy'] = KoopmanObserver(
            A, B, C_full, self.pole_mag)

        # ── Observer 1 (τ_ext insensitive): zero out τ_ext column in B ──
        # u = [τ_m, τ_ext] → zero column 1
        # Insensitive to CH0 (force sensor fault)
        B_no_tau_ext = B.copy()
        B_no_tau_ext[:, 1] = 0.0
        self.observers['tau_ext_fault'] = KoopmanObserver(
            A, B_no_tau_ext, C_full, self.pole_mag)

        # ── Observer 2 (traj_ref insensitive): tracking error monitor ──
        # CH1 is not a direct input to the Koopman model; it enters
        # indirectly via the control loop. We use a "virtual" observer
        # that monitors the full model but with a flag for tracking error.
        # For the bank structure, we create a copy of the healthy observer.
        self.observers['traj_ref_fault'] = KoopmanObserver(
            A, B, C_full, self.pole_mag)

        # ── Observer 3 (torque insensitive): zero out τ_m column in B ──
        # u = [τ_m, τ_ext] → zero column 0
        # Insensitive to CH2 (actuator fault)
        B_no_tau_m = B.copy()
        B_no_tau_m[:, 0] = 0.0
        self.observers['torque_fault'] = KoopmanObserver(
            A, B_no_tau_m, C_full, self.pole_mag)

        # ── Observer 4 (encoder insensitive): no output measurements ──
        # Runs in open-loop prediction (no correction from y).
        # Insensitive to CH3 (encoder/position sensor fault).
        # Uses a zero-row C so innovation is always 0 → pure prediction.
        C_none = np.zeros((1, self.n_z))
        # Give it a dummy single output to maintain structure
        # but the gain L will be ~0, so it runs open-loop.
        self.observers['encoder_fault'] = KoopmanObserver(
            A, B, C_none, self.pole_mag)

        # Initialize residual histories
        for name in self.observer_names:
            self.res_history[name] = deque(maxlen=self.adaptive_window)

        self.get_logger().info("Observer bank built:")
        for name, obs in self.observers.items():
            self.get_logger().info(
                f"  {name}: C={obs.C.shape}, B_cols_active="
                f"{np.count_nonzero(np.any(obs.B != 0, axis=0))}, "
                f"|L|={np.linalg.norm(obs.L):.4f}")

    # ═══════════════════════════════════════════════════════════
    # Online FDI (monitoring)
    # ═══════════════════════════════════════════════════════════

    def _state_monitoring(self):
        if self.encoder is None:
            return

        # Current state and input
        x = np.array([self.theta, self.theta_dot])
        u_raw = np.array([self.tau_m, self.tau_ext])
        u_norm = self.encoder.normalize_u(u_raw)

        # Lift state
        z_meas = self.encoder.lift(x)

        # ── Step each observer ───────────────────────────────────
        residual_norms = {}
        all_residuals = []

        for name in self.observer_names:
            obs = self.observers[name]

            # For input-insensitive observers, use modified input
            if name == 'tau_ext_fault':
                # Zero out τ_ext in the input
                u_mod = u_norm.copy()
                u_mod[1] = 0.0
                res = obs.step(z_meas, u_mod)
            elif name == 'torque_fault':
                # Zero out τ_m in the input
                u_mod = u_norm.copy()
                u_mod[0] = 0.0
                res = obs.step(z_meas, u_mod)
            else:
                res = obs.step(z_meas, u_norm)

            r_norm = np.linalg.norm(res)
            residual_norms[name] = r_norm
            all_residuals.extend(res.tolist())

            # Update adaptive stats
            if self.adaptive_thresh:
                self.res_history[name].append(r_norm)

        # ── CH1 detection: trajectory tracking error ─────────────
        # A fault on /trajectory_ref manifests as a large tracking
        # error that is NOT explained by the Koopman model
        if self.traj_ref_received:
            traj_error = abs(self.theta - self.theta_ref)
            self.traj_error_history.append(traj_error)
            residual_norms['traj_ref_fault'] = traj_error

        # ── Compute thresholds ───────────────────────────────────
        thresholds = {}
        for name in self.observer_names:
            if (self.adaptive_thresh
                    and len(self.res_history.get(name, [])) > 50):
                hist = np.array(self.res_history[name])
                mu = np.mean(hist)
                sigma = np.std(hist)
                thresholds[name] = mu + self.adaptive_sigma * max(sigma, 1e-6)
            else:
                thresholds[name] = self.det_threshold

        # For traj_ref, use tracking error stats
        if len(self.traj_error_history) > 50:
            te = np.array(self.traj_error_history)
            thresholds['traj_ref_fault'] = (
                np.mean(te) + self.adaptive_sigma * max(np.std(te), 1e-6))

        # ── Detection: healthy observer exceeds threshold ────────
        healthy_norm = residual_norms['healthy']
        fault_now = healthy_norm > thresholds.get('healthy', self.det_threshold)

        if fault_now:
            self.persist_counters['healthy'] += 1
        else:
            self.persist_counters['healthy'] = max(
                0, self.persist_counters['healthy'] - 1)

        self.fault_detected = (
            self.persist_counters['healthy'] >= self.persist_count)

        # ── Isolation ────────────────────────────────────────────
        # The observer that is INSENSITIVE to the true fault has the
        # smallest relative residual. All others are elevated.
        self.fault_channel = -1

        if self.fault_detected:
            rel_residuals = {}
            for name in self.observer_names[1:]:
                thresh = thresholds.get(name, self.det_threshold)
                rel_residuals[name] = (
                    residual_norms.get(name, 0.0) / max(thresh, 1e-8))

            # Find the observer with smallest relative residual
            min_name = min(rel_residuals, key=rel_residuals.get)
            min_val = rel_residuals[min_name]

            # Check isolation condition
            others = [v for k, v in rel_residuals.items() if k != min_name]
            if others and min_val >= 0:
                ratio = min(others) / max(min_val, 1e-8)
                if ratio > self.iso_ratio:
                    self.fault_channel = self.FAULT_CHANNELS.get(
                        min_name, -1)

        # ── Publish ──────────────────────────────────────────────

        # Residuals: [healthy, tau_ext, traj_ref, torque, encoder]
        res_msg = Float64MultiArray()
        res_msg.data = [float(residual_norms.get(n, 0.0))
                        for n in self.observer_names]
        self.pub_residuals.publish(res_msg)

        # Fault detection
        fault_msg = Float64MultiArray()
        fault_msg.data = [
            1.0 if self.fault_detected else 0.0,
            float(self.fault_channel),
            float(self.persist_counters['healthy']),
            float(healthy_norm),
            float(thresholds.get('healthy', self.det_threshold)),
        ]
        self.pub_fault.publish(fault_msg)

        # Debug: residual norms + thresholds + system state + fault info
        # Layout (22 fields):
        # [0-1]   healthy:       residual_norm, threshold
        # [2-3]   tau_ext_fault: residual_norm, threshold
        # [4-5]   traj_ref_fault: residual_norm, threshold
        # [6-7]   torque_fault:  residual_norm, threshold
        # [8-9]   encoder_fault: residual_norm, threshold
        # [10]    theta
        # [11]    theta_dot
        # [12]    tau_m
        # [13]    tau_ext
        # [14]    theta_ref
        # [15]    theta_dot_ref
        # [16]    fault_detected (0/1)
        # [17]    fault_channel (-1 if none)
        # [18]    persist_count
        # [19]    tracking_error |θ - θ_ref|
        debug_msg = Float64MultiArray()
        dd = []
        for name in self.observer_names:
            dd.append(float(residual_norms.get(name, 0.0)))
            dd.append(float(thresholds.get(name, 0.0)))
        dd.extend([
            float(self.theta),
            float(self.theta_dot),
            float(self.tau_m),
            float(self.tau_ext),
            float(self.theta_ref),
            float(self.theta_dot_ref),
            1.0 if self.fault_detected else 0.0,
            float(self.fault_channel),
            float(self.persist_counters['healthy']),
            float(abs(self.theta - self.theta_ref)),
        ])
        debug_msg.data = dd
        self.pub_debug.publish(debug_msg)


# ═══════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = KoopmanFDINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()