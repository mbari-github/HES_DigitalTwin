#!/usr/bin/env python3
"""
fdi_node.py — Neural FDI Node for Exoskeleton Digital Twin
============================================================

ROS 2 node implementing data-driven Fault Detection and Isolation
using a bank of two GRU neural observers.

Three operational phases
------------------------

PHASE 1 — COLLECT
  Subscribes to the DT topics and records healthy data into a CSV file.
  The DT must be running normally (no faults injected).
  Duration is configurable via 'collect_duration' parameter.
  After collection, transitions automatically to TRAIN.

PHASE 2 — TRAIN
  Loads the collected CSV, builds datasets, trains both GRU observers
  (Encoder + Force), and saves the trained weights.
  Training runs in a background thread to avoid blocking the ROS executor.
  After training, transitions automatically to INFERENCE.

PHASE 3 — INFERENCE
  Runs both trained observers at 200 Hz. At each step:
    1. Maintains sliding windows of inputs for each observer
    2. Runs forward pass through both GRU networks
    3. Computes residuals: r_enc = θ_meas - θ_pred
                           r_force = τ_ext_meas - τ_ext_pred
    4. Applies adaptive thresholding (rolling σ) for detection
    5. Publishes residuals and fault diagnosis

  Fault isolation logic:
    - r_enc HIGH, r_force LOW  →  Encoder fault (Ch3)
    - r_enc LOW, r_force HIGH  →  Force sensor fault (Ch0)
    - Both HIGH                →  Ambiguous (possible multiple faults)
    - Both LOW                 →  Healthy

Subscribed topics
-----------------
  /joint_states                      sensor_msgs/JointState
  /torque                            Float64Stamped
  /exo_dynamics/tau_ext_theta        Float64Stamped

Published topics
----------------
  /neural_fdi/encoder_residual       Float64     (r_enc)
  /neural_fdi/force_residual         Float64     (r_force)
  /neural_fdi/diagnosis              String      (HEALTHY / FAULT_ENCODER /
                                                   FAULT_FORCE / AMBIGUOUS)
  /neural_fdi/debug                  Float64MultiArray
      Layout: [θ_meas, θ̇_meas, τ_m, τ_ext_meas,
               θ_pred, τ_ext_pred,
               r_enc_raw, r_force_raw,
               r_enc_filtered, r_force_filtered,
               thresh_enc, thresh_force,
               diagnosis_id]
      diagnosis_id: 0=healthy, 1=fault_encoder, 2=fault_force, 3=ambiguous

Parameters
----------
  joint_name          (str)    Target joint name           default: 'rev_crank'
  collect_duration    (float)  Collection time in seconds  default: 60.0
  data_dir            (str)    Directory for CSV + models  default: '~/neural_fdi_data'
  window_size         (int)    GRU input window            default: 50
  hidden_size         (int)    GRU hidden units            default: 32
  num_layers          (int)    Stacked GRU layers          default: 2
  max_epochs          (int)    Training epochs             default: 200
  batch_size          (int)    Training batch size         default: 64
  learning_rate       (float)  Initial LR                  default: 0.001
  sigma_multiplier    (float)  Threshold = mean + k*σ      default: 5.0
  warmup_samples      (int)    Samples before detection    default: 200
  persistence_count   (int)    Consecutive alerts to confirm default: 10
  skip_collect        (bool)   Skip to TRAIN if CSV exists default: false
  skip_train          (bool)   Skip to INFERENCE if .pt exists default: false
"""

import os
import csv
import math
import threading
import time
from pathlib import Path
from enum import IntEnum
from collections import deque

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String

# Local imports (must be on PYTHONPATH or in same package)
from exoskeleton_neural_fdi.models import EncoderObserver, ForceObserver
from exoskeleton_neural_fdi.dataset import load_csv_data, build_observer_datasets
from exoskeleton_neural_fdi.trainer import train_observer

# Try importing Float64Stamped from the project's custom messages;
# fall back to std_msgs Float64 if not available.
try:
    from exoskeleton_safety_msgs.msg import Float64Stamped, Float64ArrayStamped
    USE_STAMPED = True
except ImportError:
    USE_STAMPED = False


# ── Enumerations ──

class Phase(IntEnum):
    COLLECT = 0
    TRAIN = 1
    INFERENCE = 2


class Diagnosis(IntEnum):
    HEALTHY = 0
    FAULT_ENCODER = 1
    FAULT_FORCE = 2
    AMBIGUOUS = 3


# ── Main Node ──

class NeuralFDINode(Node):

    def __init__(self):
        super().__init__('neural_fdi_node')

        # ─────────────────────────────────────────────
        # Parameters
        # ─────────────────────────────────────────────
        self.declare_parameter('joint_name', 'rev_crank')
        self.declare_parameter('collect_duration', 60.0)
        self.declare_parameter('data_dir', '~/neural_fdi_data')
        self.declare_parameter('window_size', 50)
        self.declare_parameter('hidden_size', 32)
        self.declare_parameter('num_layers', 2)
        self.declare_parameter('max_epochs', 200)
        self.declare_parameter('batch_size', 64)
        self.declare_parameter('learning_rate', 0.001)
        self.declare_parameter('sigma_multiplier', 5.0)
        self.declare_parameter('warmup_samples', 200)
        self.declare_parameter('persistence_count', 10)
        self.declare_parameter('skip_collect', False)
        self.declare_parameter('skip_train', False)

        self.joint_name = str(self.get_parameter('joint_name').value)
        self.collect_duration = float(self.get_parameter('collect_duration').value)
        self.window_size = int(self.get_parameter('window_size').value)
        self.hidden_size = int(self.get_parameter('hidden_size').value)
        self.num_layers = int(self.get_parameter('num_layers').value)

        data_dir = str(self.get_parameter('data_dir').value)
        self.data_dir = Path(os.path.expanduser(data_dir))
        self.data_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = self.data_dir / 'healthy_data.csv'
        self.enc_model_path = self.data_dir / 'encoder_observer.pt'
        self.force_model_path = self.data_dir / 'force_observer.pt'

        # ─────────────────────────────────────────────
        # State variables
        # ─────────────────────────────────────────────
        self.phase = Phase.COLLECT
        self._theta = 0.0
        self._theta_dot = 0.0
        self._tau_m = 0.0
        self._tau_ext = 0.0
        self._theta_ref = 0.0
        self._js_received = False
        self._tau_received = False
        self._tex_received = False
        self._tref_received = False

        # Collection state
        self._collect_buffer = []
        self._collect_start_time = None
        self._csv_writer = None
        self._csv_file = None

        # Training state
        self._training_thread = None
        self._training_done = False

        # Inference state
        self._encoder_obs = None
        self._force_obs = None
        self._device = torch.device('cpu')  # CPU for real-time inference

        # Sliding windows for inference (filled during INFERENCE phase)
        self._enc_window = deque(maxlen=self.window_size)    # each entry: [τ_m, τ_ext, θ_ref]
        self._force_window = deque(maxlen=self.window_size)  # each entry: [τ_m, θ, θ̇]

        # Adaptive thresholding state
        self._warmup_samples = int(self.get_parameter('warmup_samples').value)
        self._sigma_k = float(self.get_parameter('sigma_multiplier').value)
        self._persistence_count = int(self.get_parameter('persistence_count').value)

        # Rolling statistics for residuals (online Welford algorithm)
        self._enc_stats = {'n': 0, 'mean': 0.0, 'M2': 0.0}
        self._force_stats = {'n': 0, 'mean': 0.0, 'M2': 0.0}

        # EMA-filtered residuals
        self._r_enc_filtered = 0.0
        self._r_force_filtered = 0.0
        self._ema_alpha = 0.1   # EMA smoothing for residuals

        # Persistence counters
        self._enc_alarm_count = 0
        self._force_alarm_count = 0

        # Current diagnosis
        self._diagnosis = Diagnosis.HEALTHY
        self._inference_step = 0

        # ─────────────────────────────────────────────
        # ROS 2 Subscribers
        # ─────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, qos
        )

        if USE_STAMPED:
            self.sub_tau = self.create_subscription(
                Float64Stamped, '/torque', self._torque_stamped_cb, qos
            )
            self.sub_tex = self.create_subscription(
                Float64Stamped, '/exo_dynamics/tau_ext_theta',
                self._tau_ext_stamped_cb, qos
            )
            self.sub_tref = self.create_subscription(
                Float64ArrayStamped, '/trajectory_ref',
                self._traj_ref_stamped_cb, qos
            )
        else:
            self.sub_tau = self.create_subscription(
                Float64, '/torque', self._torque_cb, qos
            )
            self.sub_tex = self.create_subscription(
                Float64, '/exo_dynamics/tau_ext_theta',
                self._tau_ext_cb, qos
            )
            self.sub_tref = self.create_subscription(
                Float64MultiArray, '/trajectory_ref',
                self._traj_ref_cb, qos
            )

        # ─────────────────────────────────────────────
        # ROS 2 Publishers
        # ─────────────────────────────────────────────
        self.pub_r_enc = self.create_publisher(Float64, '/neural_fdi/encoder_residual', 10)
        self.pub_r_force = self.create_publisher(Float64, '/neural_fdi/force_residual', 10)
        self.pub_diag = self.create_publisher(String, '/neural_fdi/diagnosis', 10)
        self.pub_debug = self.create_publisher(
            Float64MultiArray, '/neural_fdi/debug', 10
        )

        # ─────────────────────────────────────────────
        # Main timer at 200 Hz
        # ─────────────────────────────────────────────
        self.timer = self.create_timer(0.005, self._update)

        # ─────────────────────────────────────────────
        # Phase initialization
        # ─────────────────────────────────────────────
        skip_collect = bool(self.get_parameter('skip_collect').value)
        skip_train = bool(self.get_parameter('skip_train').value)

        if skip_train and self.enc_model_path.exists() and self.force_model_path.exists():
            self.get_logger().info("skip_train=true, loading existing models...")
            self.phase = Phase.INFERENCE
            self._load_models()
        elif skip_collect and self.csv_path.exists():
            self.get_logger().info("skip_collect=true, CSV found, starting TRAIN...")
            self.phase = Phase.TRAIN
            self._start_training()
        else:
            self.get_logger().info(
                f"Phase: COLLECT | Duration: {self.collect_duration}s | "
                f"Saving to: {self.csv_path}"
            )
            self._init_csv()

    # ═════════════════════════════════════════════════
    # SUBSCRIBER CALLBACKS
    # ═════════════════════════════════════════════════

    def _joint_states_cb(self, msg: JointState):
        """Extract θ and θ̇ for the target joint."""
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return
        if idx < len(msg.position):
            self._theta = msg.position[idx]
        if idx < len(msg.velocity):
            self._theta_dot = msg.velocity[idx]
        self._js_received = True

    def _torque_stamped_cb(self, msg):
        self._tau_m = msg.data
        self._tau_received = True

    def _torque_cb(self, msg: Float64):
        self._tau_m = msg.data
        self._tau_received = True

    def _tau_ext_stamped_cb(self, msg):
        self._tau_ext = msg.data
        self._tex_received = True

    def _tau_ext_cb(self, msg: Float64):
        self._tau_ext = msg.data
        self._tex_received = True

    def _traj_ref_stamped_cb(self, msg):
        """Extract θ_ref from trajectory reference [θ_ref, θ̇_ref, θ̈_ref]."""
        if len(msg.data) >= 1:
            self._theta_ref = msg.data[0]
        self._tref_received = True

    def _traj_ref_cb(self, msg: Float64MultiArray):
        """Extract θ_ref from trajectory reference [θ_ref, θ̇_ref, θ̈_ref]."""
        if len(msg.data) >= 1:
            self._theta_ref = msg.data[0]
        self._tref_received = True

    # ═════════════════════════════════════════════════
    # MAIN UPDATE LOOP (200 Hz)
    # ═════════════════════════════════════════════════

    def _update(self):
        """Dispatch to current phase handler."""
        if self.phase == Phase.COLLECT:
            self._update_collect()
        elif self.phase == Phase.TRAIN:
            self._update_train()
        elif self.phase == Phase.INFERENCE:
            self._update_inference()

    # ─────────────────────────────────────────────────
    # PHASE 1: COLLECT
    # ─────────────────────────────────────────────────

    def _init_csv(self):
        """Open CSV file and write header."""
        self._csv_file = open(self.csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp', 'theta', 'theta_dot', 'tau_m', 'tau_ext', 'theta_ref'
        ])

    def _update_collect(self):
        """Record one sample to CSV."""
        if not (self._js_received and self._tau_received and self._tex_received and self._tref_received):
            return  # wait for all topics

        now = self.get_clock().now()

        if self._collect_start_time is None:
            self._collect_start_time = now
            self.get_logger().info("COLLECT: first sample received, recording...")

        elapsed = (now - self._collect_start_time).nanoseconds * 1e-9

        # Write sample
        self._csv_writer.writerow([
            f"{elapsed:.6f}",
            f"{self._theta:.8f}",
            f"{self._theta_dot:.8f}",
            f"{self._tau_m:.8f}",
            f"{self._tau_ext:.8f}",
            f"{self._theta_ref:.8f}",
        ])

        # Check if collection is done
        if elapsed >= self.collect_duration:
            self._csv_file.close()
            n_samples = int(elapsed * 200)
            self.get_logger().info(
                f"COLLECT complete: {elapsed:.1f}s, ~{n_samples} samples "
                f"saved to {self.csv_path}"
            )
            self.phase = Phase.TRAIN
            self._start_training()

    # ─────────────────────────────────────────────────
    # PHASE 2: TRAIN
    # ─────────────────────────────────────────────────

    def _start_training(self):
        """Launch training in a background thread."""
        self.get_logger().info("Phase: TRAIN | Starting background training...")
        self._training_done = False
        self._training_thread = threading.Thread(
            target=self._training_worker, daemon=True
        )
        self._training_thread.start()

    def _training_worker(self):
        """
        Background thread: loads CSV, trains both observers, saves weights.
        Runs entirely off the ROS executor thread.
        """
        try:
            # Load data
            raw = load_csv_data(str(self.csv_path))
            N = len(raw['theta'])
            self.get_logger().info(f"TRAIN: Loaded {N} samples from CSV")

            window = self.window_size
            batch_size = int(self.get_parameter('batch_size').value)
            max_epochs = int(self.get_parameter('max_epochs').value)
            lr = float(self.get_parameter('learning_rate').value)
            hidden = self.hidden_size
            layers = self.num_layers

            # Build datasets
            datasets = build_observer_datasets(raw, window_size=window)

            from torch.utils.data import DataLoader

            enc_train_dl = DataLoader(
                datasets['encoder_train'], batch_size=batch_size,
                shuffle=True, num_workers=0,
            )
            enc_val_dl = DataLoader(
                datasets['encoder_val'], batch_size=batch_size,
                shuffle=False, num_workers=0,
            )
            force_train_dl = DataLoader(
                datasets['force_train'], batch_size=batch_size,
                shuffle=True, num_workers=0,
            )
            force_val_dl = DataLoader(
                datasets['force_val'], batch_size=batch_size,
                shuffle=False, num_workers=0,
            )

            device = self._device

            # ── Train Encoder Observer ──
            enc_model = EncoderObserver(
                input_size=3, hidden_size=hidden, num_layers=layers
            )
            enc_in_m, enc_in_s = datasets['norm_encoder_input']
            enc_out_m, enc_out_s = datasets['norm_encoder_output']
            enc_model.set_normalization(
                torch.from_numpy(enc_in_m).float(),
                torch.from_numpy(enc_in_s).float(),
                torch.from_numpy(enc_out_m).float(),
                torch.from_numpy(enc_out_s).float(),
            )
            enc_result = train_observer(
                enc_model, enc_train_dl, enc_val_dl, device,
                name="Encoder", max_epochs=max_epochs, lr=lr,
            )

            # ── Train Force Observer ──
            force_model = ForceObserver(
                input_size=3, hidden_size=hidden, num_layers=layers
            )
            f_in_m, f_in_s = datasets['norm_force_input']
            f_out_m, f_out_s = datasets['norm_force_output']
            force_model.set_normalization(
                torch.from_numpy(f_in_m).float(),
                torch.from_numpy(f_in_s).float(),
                torch.from_numpy(f_out_m).float(),
                torch.from_numpy(f_out_s).float(),
            )
            force_result = train_observer(
                force_model, force_train_dl, force_val_dl, device,
                name="Force", max_epochs=max_epochs, lr=lr,
            )

            # ── Save models ──
            torch.save({
                'model_state_dict': enc_model.state_dict(),
                'config': {
                    'input_size': 3, 'hidden_size': hidden,
                    'num_layers': layers, 'window_size': window,
                },
                'best_val_loss': enc_result['best_val_loss'],
            }, self.enc_model_path)

            torch.save({
                'model_state_dict': force_model.state_dict(),
                'config': {
                    'input_size': 3, 'hidden_size': hidden,
                    'num_layers': layers, 'window_size': window,
                },
                'best_val_loss': force_result['best_val_loss'],
            }, self.force_model_path)

            self.get_logger().info(
                f"TRAIN complete | Encoder val_loss={enc_result['best_val_loss']:.6f} "
                f"| Force val_loss={force_result['best_val_loss']:.6f}"
            )

            self._training_done = True

        except Exception as e:
            self.get_logger().error(f"TRAIN failed: {e}")
            import traceback
            traceback.print_exc()

    def _update_train(self):
        """Poll for training completion."""
        if self._training_done:
            self.get_logger().info("Phase: INFERENCE | Loading trained models...")
            self._load_models()
            self.phase = Phase.INFERENCE

    # ─────────────────────────────────────────────────
    # PHASE 3: INFERENCE
    # ─────────────────────────────────────────────────

    def _load_models(self):
        """Load trained model weights from disk."""
        # ── Encoder Observer ──
        enc_ckpt = torch.load(self.enc_model_path, map_location='cpu', weights_only=False)
        cfg = enc_ckpt['config']
        self._encoder_obs = EncoderObserver(
            input_size=cfg['input_size'],
            hidden_size=cfg['hidden_size'],
            num_layers=cfg['num_layers'],
        )
        self._encoder_obs.load_state_dict(enc_ckpt['model_state_dict'])
        self._encoder_obs.eval()
        self._encoder_obs.to(self._device)

        # Update window size from saved config
        self.window_size = cfg['window_size']
        self._enc_window = deque(maxlen=self.window_size)
        self._force_window = deque(maxlen=self.window_size)

        # ── Force Observer ──
        force_ckpt = torch.load(self.force_model_path, map_location='cpu', weights_only=False)
        cfg_f = force_ckpt['config']
        self._force_obs = ForceObserver(
            input_size=cfg_f['input_size'],
            hidden_size=cfg_f['hidden_size'],
            num_layers=cfg_f['num_layers'],
        )
        self._force_obs.load_state_dict(force_ckpt['model_state_dict'])
        self._force_obs.eval()
        self._force_obs.to(self._device)

        self.get_logger().info(
            f"Models loaded | window={self.window_size} | "
            f"Encoder: {sum(p.numel() for p in self._encoder_obs.parameters())} params | "
            f"Force: {sum(p.numel() for p in self._force_obs.parameters())} params"
        )

    @torch.no_grad()
    def _update_inference(self):
        """Run both observers, compute residuals, detect and isolate faults."""
        if not (self._js_received and self._tau_received and self._tex_received and self._tref_received):
            return

        # ── Update sliding windows ──
        self._enc_window.append([self._tau_m, self._tau_ext, self._theta_ref])
        self._force_window.append([self._tau_m, self._theta, self._theta_dot])

        # Wait until windows are full
        if len(self._enc_window) < self.window_size:
            return

        self._inference_step += 1

        # ── Build tensors ──
        enc_input = torch.tensor(
            list(self._enc_window), dtype=torch.float32
        ).unsqueeze(0).to(self._device)            # (1, W, 2)

        force_input = torch.tensor(
            list(self._force_window), dtype=torch.float32
        ).unsqueeze(0).to(self._device)             # (1, W, 3)

        # ── Forward pass ──
        theta_pred = self._encoder_obs(enc_input).item()
        tau_ext_pred = self._force_obs(force_input).item()

        # ── Raw residuals ──
        r_enc_raw = self._theta - theta_pred
        r_force_raw = self._tau_ext - tau_ext_pred

        # ── EMA filtering ──
        alpha = self._ema_alpha
        self._r_enc_filtered = alpha * abs(r_enc_raw) + (1 - alpha) * self._r_enc_filtered
        self._r_force_filtered = alpha * abs(r_force_raw) + (1 - alpha) * self._r_force_filtered

        # ── Adaptive thresholding (Welford online algorithm) ──
        # CRITICAL: statistics are updated ONLY when the system is HEALTHY.
        # If we keep updating during a fault, the mean and std absorb the
        # fault signature, the threshold rises, and the fault becomes the
        # "new normal" — causing missed detections and false recoveries.
        is_healthy = (self._diagnosis == Diagnosis.HEALTHY)

        if self._inference_step <= self._warmup_samples:
            # During warmup, always accumulate (no fault possible yet)
            self._welford_update(self._enc_stats, abs(r_enc_raw))
            self._welford_update(self._force_stats, abs(r_force_raw))

            thresh_enc = float('inf')
            thresh_force = float('inf')
        else:
            # After warmup: update stats ONLY if currently healthy
            if is_healthy:
                self._welford_update(self._enc_stats, abs(r_enc_raw))
                self._welford_update(self._force_stats, abs(r_force_raw))

            enc_mean, enc_std = self._welford_stats(self._enc_stats)
            force_mean, force_std = self._welford_stats(self._force_stats)

            thresh_enc = enc_mean + self._sigma_k * enc_std
            thresh_force = force_mean + self._sigma_k * force_std

        # ── Detection with persistence ──
        enc_alert = self._r_enc_filtered > thresh_enc
        force_alert = self._r_force_filtered > thresh_force

        if enc_alert:
            self._enc_alarm_count = min(
                self._enc_alarm_count + 1, self._persistence_count * 2
            )
        else:
            self._enc_alarm_count = max(self._enc_alarm_count - 1, 0)

        if force_alert:
            self._force_alarm_count = min(
                self._force_alarm_count + 1, self._persistence_count * 2
            )
        else:
            self._force_alarm_count = max(self._force_alarm_count - 1, 0)

        enc_confirmed = self._enc_alarm_count >= self._persistence_count
        force_confirmed = self._force_alarm_count >= self._persistence_count

        # ── Isolation logic ──
        if enc_confirmed and force_confirmed:
            self._diagnosis = Diagnosis.AMBIGUOUS
        elif enc_confirmed:
            self._diagnosis = Diagnosis.FAULT_ENCODER
        elif force_confirmed:
            self._diagnosis = Diagnosis.FAULT_FORCE
        else:
            self._diagnosis = Diagnosis.HEALTHY

        # ── Publish ──
        # Residuals
        msg_r_enc = Float64()
        msg_r_enc.data = r_enc_raw
        self.pub_r_enc.publish(msg_r_enc)

        msg_r_force = Float64()
        msg_r_force.data = r_force_raw
        self.pub_r_force.publish(msg_r_force)

        # Diagnosis string
        diag_msg = String()
        diag_names = {
            Diagnosis.HEALTHY: 'HEALTHY',
            Diagnosis.FAULT_ENCODER: 'FAULT_ENCODER',
            Diagnosis.FAULT_FORCE: 'FAULT_FORCE',
            Diagnosis.AMBIGUOUS: 'AMBIGUOUS',
        }
        diag_msg.data = diag_names[self._diagnosis]
        self.pub_diag.publish(diag_msg)

        # Debug vector
        debug_msg = Float64MultiArray()
        debug_msg.data = [
            self._theta,                    # [0] θ_meas
            self._theta_dot,                # [1] θ̇_meas
            self._tau_m,                    # [2] τ_m
            self._tau_ext,                  # [3] τ_ext_meas
            theta_pred,                     # [4] θ_pred
            tau_ext_pred,                   # [5] τ_ext_pred
            r_enc_raw,                      # [6] r_enc raw
            r_force_raw,                    # [7] r_force raw
            self._r_enc_filtered,           # [8] r_enc filtered (EMA)
            self._r_force_filtered,         # [9] r_force filtered (EMA)
            thresh_enc if thresh_enc != float('inf') else -1.0,   # [10]
            thresh_force if thresh_force != float('inf') else -1.0, # [11]
            float(self._diagnosis),         # [12] diagnosis_id
        ]
        self.pub_debug.publish(debug_msg)

    # ─────────────────────────────────────────────────
    # Utility: Welford online statistics
    # ─────────────────────────────────────────────────

    @staticmethod
    def _welford_update(stats: dict, value: float):
        """Online mean/variance update (Welford's algorithm)."""
        stats['n'] += 1
        delta = value - stats['mean']
        stats['mean'] += delta / stats['n']
        delta2 = value - stats['mean']
        stats['M2'] += delta * delta2

    @staticmethod
    def _welford_stats(stats: dict):
        """Return (mean, std) from Welford state."""
        if stats['n'] < 2:
            return stats['mean'], 1e-8
        variance = stats['M2'] / (stats['n'] - 1)
        return stats['mean'], math.sqrt(max(variance, 1e-16))


# ═════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = NeuralFDINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()