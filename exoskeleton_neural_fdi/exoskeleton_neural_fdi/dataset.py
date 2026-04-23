"""
dataset.py — Sliding Window Dataset for GRU Observer Training
==============================================================

Converts raw time-series data collected from the Digital Twin into
windowed sequences suitable for training the two GRU observers.

Data format expected from the collector
---------------------------------------
A CSV file (or numpy arrays) with columns recorded at 200 Hz:

    timestamp, theta, theta_dot, tau_m, tau_ext

Each row is one sample at dt = 5 ms.

Windowing strategy
------------------
For a window size W, at each timestep t we create:

  Observer A (encoder):
    input  = [τ_m, τ_ext](t-W+1 : t+1)   shape (W, 2)
    target = θ(t)                           shape (1,)

  Observer B (force):
    input  = [τ_m, θ, θ̇](t-W+1 : t+1)    shape (W, 3)
    target = τ_ext(t)                       shape (1,)

The window slides by 1 sample (stride=1), producing N - W + 1
training examples from N raw samples.
"""

import numpy as np
import torch
from torch.utils.data import Dataset
from pathlib import Path
from typing import Tuple, Optional


class SlidingWindowDataset(Dataset):
    """
    Generic sliding-window dataset for time-series observer training.

    Given raw arrays of inputs and targets, produces (window, target)
    pairs with configurable window size and stride.
    """

    def __init__(
        self,
        inputs: np.ndarray,     # (N, n_features) — input time series
        targets: np.ndarray,    # (N, 1) or (N,) — target time series
        window_size: int = 50,  # number of past samples in each window
        stride: int = 1,        # sliding stride (1 = every sample)
    ):
        """
        Args:
            inputs:      Raw input features, shape (N, n_features)
            targets:     Raw target values, shape (N,) or (N, 1)
            window_size: Number of timesteps in each input window
            stride:      Step between consecutive windows
        """
        assert len(inputs) == len(targets), \
            f"Input length {len(inputs)} != target length {len(targets)}"
        assert window_size > 0, f"Window size must be > 0, got {window_size}"
        assert len(inputs) >= window_size, \
            f"Data length {len(inputs)} < window size {window_size}"

        self.inputs = inputs.astype(np.float32)
        self.targets = targets.reshape(-1, 1).astype(np.float32)
        self.window_size = window_size
        self.stride = stride

        # Precompute valid indices
        self.indices = list(range(
            window_size - 1,       # first valid index (end of first full window)
            len(inputs),           # last valid index + 1
            stride,
        ))

    def __len__(self) -> int:
        return len(self.indices)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Returns:
            x: (window_size, n_features) — input window
            y: (1,) — target at the end of the window
        """
        t = self.indices[idx]
        x = self.inputs[t - self.window_size + 1: t + 1]   # (W, n_feat)
        y = self.targets[t]                                  # (1,)
        return torch.from_numpy(x), torch.from_numpy(y)


def compute_normalization(data: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute per-feature mean and std for normalization.

    Args:
        data: (N, n_features) array

    Returns:
        mean: (n_features,)
        std:  (n_features,) — clipped to min 1e-8 to avoid division by zero
    """
    mean = data.mean(axis=0)
    std = data.std(axis=0)
    std = np.clip(std, 1e-8, None)
    return mean, std


def load_csv_data(csv_path: str) -> dict:
    """
    Load collected data from CSV file.

    Expected CSV format (header row):
        timestamp,theta,theta_dot,tau_m,tau_ext,theta_ref

    Returns:
        Dictionary with numpy arrays for each signal.
    """
    path = Path(csv_path)
    if not path.exists():
        raise FileNotFoundError(f"Data file not found: {csv_path}")

    data = np.genfromtxt(csv_path, delimiter=',', names=True)

    return {
        'timestamp': data['timestamp'],
        'theta': data['theta'],
        'theta_dot': data['theta_dot'],
        'tau_m': data['tau_m'],
        'tau_ext': data['tau_ext'],
        'theta_ref': data['theta_ref'],
    }


def build_observer_datasets(
    raw: dict,
    window_size: int = 50,
    stride: int = 1,
    val_fraction: float = 0.2,
) -> dict:
    """
    Build training and validation datasets for both observers
    from raw collected data.

    Args:
        raw:           Dictionary with keys 'theta', 'theta_dot', 'tau_m', 'tau_ext'
        window_size:   Sliding window length
        stride:        Window stride
        val_fraction:  Fraction of data reserved for validation (taken from the END,
                       not random, to preserve temporal structure)

    Returns:
        Dictionary containing:
          'encoder_train', 'encoder_val':  SlidingWindowDataset for Observer A
          'force_train', 'force_val':      SlidingWindowDataset for Observer B
          'norm_encoder_input':   (mean, std) for Observer A inputs
          'norm_encoder_output':  (mean, std) for Observer A targets
          'norm_force_input':     (mean, std) for Observer B inputs
          'norm_force_output':    (mean, std) for Observer B targets
    """
    N = len(raw['theta'])
    split_idx = int(N * (1.0 - val_fraction))

    # ── Observer A: Encoder Observer ──
    # Input:  [τ_m, τ_ext, θ_ref]
    # Target: θ
    enc_inputs = np.column_stack([raw['tau_m'], raw['tau_ext'], raw['theta_ref']])
    enc_targets = raw['theta']

    # ── Observer B: Force Observer ──
    # Input:  [τ_m, θ, θ̇]
    # Target: τ_ext
    force_inputs = np.column_stack([raw['tau_m'], raw['theta'], raw['theta_dot']])
    force_targets = raw['tau_ext']

    # ── Compute normalization on TRAINING split only ──
    enc_in_mean, enc_in_std = compute_normalization(enc_inputs[:split_idx])
    enc_out_mean, enc_out_std = compute_normalization(
        enc_targets[:split_idx].reshape(-1, 1)
    )
    force_in_mean, force_in_std = compute_normalization(force_inputs[:split_idx])
    force_out_mean, force_out_std = compute_normalization(
        force_targets[:split_idx].reshape(-1, 1)
    )

    # ── Build datasets ──
    result = {
        # Datasets
        'encoder_train': SlidingWindowDataset(
            enc_inputs[:split_idx], enc_targets[:split_idx],
            window_size, stride
        ),
        'encoder_val': SlidingWindowDataset(
            enc_inputs[split_idx:], enc_targets[split_idx:],
            window_size, stride
        ),
        'force_train': SlidingWindowDataset(
            force_inputs[:split_idx], force_targets[:split_idx],
            window_size, stride
        ),
        'force_val': SlidingWindowDataset(
            force_inputs[split_idx:], force_targets[split_idx:],
            window_size, stride
        ),

        # Normalization stats
        'norm_encoder_input': (enc_in_mean, enc_in_std),
        'norm_encoder_output': (enc_out_mean, enc_out_std),
        'norm_force_input': (force_in_mean, force_in_std),
        'norm_force_output': (force_out_mean, force_out_std),
    }

    return result