"""
models.py — GRU-based Neural Observers for Sensor FDI
======================================================

Two observer networks, each structured for isolation of a specific
sensor fault on the hand exoskeleton Digital Twin.

System recap
------------
  State:   x  = [θ, θ̇]          (encoder, /joint_states)
  Inputs:  u  = [τ_m, τ_ext]     (/torque, /exo_dynamics/tau_ext_theta)
  Faults:  Ch0 = load cell (τ_ext),  Ch3 = encoder (θ, θ̇)

Observer A  —  Encoder Observer
  Input:   [τ_m, τ_ext_meas]     (motor torque + measured external force)
  Output:  θ_pred                 (predicted crank angle)
  Residual: r_enc = θ_meas - θ_pred
  Sensitive to: encoder fault (Ch3)
  Robust to:    force sensor fault (Ch0 affects input, but effect on θ
                is attenuated by plant inertia → small indirect effect)

Observer B  —  Force Observer
  Input:   [τ_m, θ_meas, θ̇_meas]  (motor torque + measured state)
  Output:  τ_ext_pred              (predicted external force)
  Residual: r_force = τ_ext_meas - τ_ext_pred
  Sensitive to: force sensor fault (Ch0)
  Robust to:    encoder fault (Ch3 affects input, but propagation through
                closed-loop dynamics introduces a detectable delay)

Architecture choice: GRU
-------------------------
GRU (Gated Recurrent Unit) over LSTM because:
  - Fewer parameters (2 gates vs 3) → faster training and inference
  - Comparable performance on short sequences
  - Better suited for 200 Hz real-time inference on the DT
  - Cho et al. (2014) showed GRU matches LSTM on most sequence tasks

Each network processes a sliding window of W past samples and predicts
the CURRENT output (not the next step). This is a reconstruction task,
not a forecasting task — the residual measures whether the current
measurement is consistent with the recent input history.
"""

import torch
import torch.nn as nn


class EncoderObserver(nn.Module):
    """
    Observer A: predicts θ from [τ_m, τ_ext_meas, θ_ref] history.

    Architecture:
        [τ_m(t-W:t), τ_ext(t-W:t), θ_ref(t-W:t)]  →  GRU  →  FC  →  θ_pred(t)
                         (W × 3)

    The inclusion of θ_ref is critical: it provides an independent
    anchor that the GRU can use to detect encoder offsets. Without it,
    the GRU adapts to the corrupted θ through the closed-loop inputs
    and the residual vanishes. With θ_ref, the network learns that
    θ_meas should track θ_ref closely — an encoder offset breaks
    this relationship and produces a persistent residual.
    """

    def __init__(
        self,
        input_size: int = 3,       # [τ_m, τ_ext, θ_ref]
        hidden_size: int = 32,     # GRU hidden units
        num_layers: int = 2,       # stacked GRU layers
        dropout: float = 0.1,      # dropout between GRU layers
    ):
        super().__init__()

        self.input_size = input_size
        self.hidden_size = hidden_size
        self.num_layers = num_layers

        # --- Input normalization (populated during training) ---
        # These are registered as buffers so they get saved/loaded
        # with the model and moved to the correct device automatically.
        self.register_buffer('input_mean', torch.zeros(input_size))
        self.register_buffer('input_std', torch.ones(input_size))
        self.register_buffer('output_mean', torch.zeros(1))
        self.register_buffer('output_std', torch.ones(1))

        # --- GRU encoder ---
        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,        # input shape: (batch, seq, features)
            dropout=dropout if num_layers > 1 else 0.0,
        )

        # --- Output head ---
        # Takes the last hidden state and maps it to a scalar prediction.
        # Two-layer FC with ReLU for mild nonlinearity.
        self.fc = nn.Sequential(
            nn.Linear(hidden_size, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass.

        Args:
            x: (batch, window_size, input_size) — input sequence

        Returns:
            y_pred: (batch, 1) — predicted θ at current timestep
        """
        # Normalize input
        x_norm = (x - self.input_mean) / (self.input_std + 1e-8)

        # GRU: process the full sequence
        # gru_out shape: (batch, seq_len, hidden_size)
        # h_n shape: (num_layers, batch, hidden_size)
        gru_out, h_n = self.gru(x_norm)

        # Take the output at the LAST timestep (current time)
        last_output = gru_out[:, -1, :]   # (batch, hidden_size)

        # Map to prediction (normalized output space)
        y_pred_norm = self.fc(last_output)   # (batch, 1)

        # Denormalize output
        y_pred = y_pred_norm * (self.output_std + 1e-8) + self.output_mean

        return y_pred

    def set_normalization(
        self,
        input_mean: torch.Tensor,
        input_std: torch.Tensor,
        output_mean: torch.Tensor,
        output_std: torch.Tensor,
    ):
        """Set normalization statistics from training data."""
        self.input_mean.copy_(input_mean)
        self.input_std.copy_(input_std)
        self.output_mean.copy_(output_mean)
        self.output_std.copy_(output_std)


class ForceObserver(nn.Module):
    """
    Observer B: predicts τ_ext from [τ_m, θ_meas, θ̇_meas] history.

    Architecture:
        [τ_m(t-W:t), θ(t-W:t), θ̇(t-W:t)]  →  GRU  →  FC  →  τ_ext_pred(t)
                         (W × 3)

    This observer essentially learns the inverse dynamics:
        τ_ext = M_eff·θ̈ + proj + τ_fric + τ_damp - τ_m - τ_pass

    but without needing the analytical model. The GRU implicitly
    estimates θ̈ from the θ̇ sequence and learns the nonlinear
    friction/inertia terms from data.
    """

    def __init__(
        self,
        input_size: int = 3,       # [τ_m, θ, θ̇]
        hidden_size: int = 32,     # GRU hidden units
        num_layers: int = 2,       # stacked GRU layers
        dropout: float = 0.1,      # dropout between GRU layers
    ):
        super().__init__()

        self.input_size = input_size
        self.hidden_size = hidden_size
        self.num_layers = num_layers

        # --- Input normalization ---
        self.register_buffer('input_mean', torch.zeros(input_size))
        self.register_buffer('input_std', torch.ones(input_size))
        self.register_buffer('output_mean', torch.zeros(1))
        self.register_buffer('output_std', torch.ones(1))

        # --- GRU encoder ---
        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0,
        )

        # --- Output head ---
        self.fc = nn.Sequential(
            nn.Linear(hidden_size, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass.

        Args:
            x: (batch, window_size, input_size) — input sequence

        Returns:
            y_pred: (batch, 1) — predicted τ_ext at current timestep
        """
        x_norm = (x - self.input_mean) / (self.input_std + 1e-8)

        gru_out, h_n = self.gru(x_norm)
        last_output = gru_out[:, -1, :]
        y_pred_norm = self.fc(last_output)
        y_pred = y_pred_norm * (self.output_std + 1e-8) + self.output_mean

        return y_pred

    def set_normalization(
        self,
        input_mean: torch.Tensor,
        input_std: torch.Tensor,
        output_mean: torch.Tensor,
        output_std: torch.Tensor,
    ):
        """Set normalization statistics from training data."""
        self.input_mean.copy_(input_mean)
        self.input_std.copy_(input_std)
        self.output_mean.copy_(output_mean)
        self.output_std.copy_(output_std)