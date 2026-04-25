"""
trainer.py — Offline Training Script for GRU Neural Observers
==============================================================

Loads healthy data from CSV, trains both observers (Encoder + Force),
saves trained weights and normalization statistics.

Usage
-----
    python3 trainer.py --data healthy_data.csv --output ./trained_models/

This script is designed to run OFFLINE (not inside ROS 2). It:
  1. Loads CSV data collected by the FDI node's COLLECT phase
  2. Splits into train/validation (80/20, temporal split)
  3. Trains Observer A (encoder) and Observer B (force)
  4. Saves model weights + normalization stats as .pt files

Training details
----------------
  - Loss: MSE (mean squared error)
  - Optimizer: Adam with weight decay (AdamW)
  - Scheduler: ReduceLROnPlateau (patience=10, factor=0.5)
  - Early stopping: patience=20 epochs on validation loss
  - Gradient clipping: max_norm=1.0 (prevents exploding gradients in RNNs)
"""

import argparse
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from exoskeleton_neural_fdi.models import EncoderObserver, ForceObserver
from exoskeleton_neural_fdi.dataset import load_csv_data, build_observer_datasets


def train_one_epoch(
    model: nn.Module,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    criterion: nn.Module,
    device: torch.device,
    max_grad_norm: float = 1.0,
) -> float:
    """Train for one epoch, return average loss."""
    model.train()
    total_loss = 0.0
    n_batches = 0

    for x_batch, y_batch in loader:
        x_batch = x_batch.to(device)
        y_batch = y_batch.to(device)

        optimizer.zero_grad()
        y_pred = model(x_batch)
        loss = criterion(y_pred, y_batch)
        loss.backward()

        # Gradient clipping — critical for RNNs
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_grad_norm)

        optimizer.step()

        total_loss += loss.item()
        n_batches += 1

    return total_loss / max(n_batches, 1)


@torch.no_grad()
def validate(
    model: nn.Module,
    loader: DataLoader,
    criterion: nn.Module,
    device: torch.device,
) -> float:
    """Validate, return average loss."""
    model.eval()
    total_loss = 0.0
    n_batches = 0

    for x_batch, y_batch in loader:
        x_batch = x_batch.to(device)
        y_batch = y_batch.to(device)

        y_pred = model(x_batch)
        loss = criterion(y_pred, y_batch)

        total_loss += loss.item()
        n_batches += 1

    return total_loss / max(n_batches, 1)


def train_observer(
    model: nn.Module,
    train_loader: DataLoader,
    val_loader: DataLoader,
    device: torch.device,
    name: str,
    max_epochs: int = 200,
    lr: float = 1e-3,
    weight_decay: float = 1e-4,
    patience: int = 20,
    min_delta: float = 1e-6,
) -> dict:
    """
    Full training loop for one observer with early stopping.

    Args:
        model:        The GRU observer network
        train_loader: Training data
        val_loader:   Validation data
        device:       CPU or CUDA
        name:         Observer name for logging (e.g. "Encoder" or "Force")
        max_epochs:   Maximum number of training epochs
        lr:           Initial learning rate
        weight_decay: AdamW weight decay
        patience:     Early stopping patience (epochs without improvement)
        min_delta:    Minimum improvement to count as progress

    Returns:
        Dictionary with training history and best model state_dict.
    """
    model = model.to(device)
    criterion = nn.MSELoss()
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=lr, weight_decay=weight_decay
    )
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=10
    )

    best_val_loss = float('inf')
    best_state = None
    epochs_without_improvement = 0
    history = {'train_loss': [], 'val_loss': [], 'lr': []}

    print(f"\n{'='*60}", flush=True)
    print(f"  Training {name} Observer", flush=True)
    print(f"{'='*60}", flush=True)
    print(f"  Model params: {sum(p.numel() for p in model.parameters()):,}", flush=True)
    print(f"  Train samples: {len(train_loader.dataset):,}", flush=True)
    print(f"  Val samples:   {len(val_loader.dataset):,}", flush=True)
    print(f"  Device: {device}", flush=True)
    print(f"{'='*60}\n", flush=True)

    t_start = time.time()

    for epoch in range(max_epochs):
        train_loss = train_one_epoch(
            model, train_loader, optimizer, criterion, device
        )
        val_loss = validate(model, val_loader, criterion, device)

        current_lr = optimizer.param_groups[0]['lr']
        scheduler.step(val_loss)

        history['train_loss'].append(train_loss)
        history['val_loss'].append(val_loss)
        history['lr'].append(current_lr)

        # Early stopping check
        if val_loss < best_val_loss - min_delta:
            best_val_loss = val_loss
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}
            epochs_without_improvement = 0
            marker = ' *'   # mark improvement
        else:
            epochs_without_improvement += 1
            marker = ''

        # Print progress every 10 epochs or on improvement
        if (epoch + 1) % 10 == 0 or marker:
            print(
                f"  [{name}] Epoch {epoch+1:3d}/{max_epochs} | "
                f"train={train_loss:.6f} | val={val_loss:.6f} | "
                f"lr={current_lr:.2e}{marker}",
                flush=True
            )

        if epochs_without_improvement >= patience:
            print(f"\n  Early stopping at epoch {epoch+1} "
                  f"(no improvement for {patience} epochs)", flush=True)
            break

    elapsed = time.time() - t_start
    print(f"\n  Training complete in {elapsed:.1f}s", flush=True)
    print(f"  Best val loss: {best_val_loss:.6f}", flush=True)

    # Restore best model
    if best_state is not None:
        model.load_state_dict(best_state)

    return {
        'history': history,
        'best_val_loss': best_val_loss,
        'best_state': best_state,
    }


def main():
    parser = argparse.ArgumentParser(
        description='Train GRU Neural Observers for Sensor FDI'
    )
    parser.add_argument(
        '--data', type=str, required=True,
        help='Path to CSV file with healthy data '
             '(columns: timestamp, theta, theta_dot, tau_m, tau_ext)'
    )
    parser.add_argument(
        '--output', type=str, default='./trained_models',
        help='Directory to save trained model weights'
    )
    parser.add_argument(
        '--window', type=int, default=50,
        help='Sliding window size (samples). Default=50 (250ms at 200Hz)'
    )
    parser.add_argument(
        '--batch-size', type=int, default=64,
        help='Training batch size'
    )
    parser.add_argument(
        '--epochs', type=int, default=200,
        help='Maximum training epochs'
    )
    parser.add_argument(
        '--lr', type=float, default=1e-3,
        help='Initial learning rate'
    )
    parser.add_argument(
        '--hidden', type=int, default=32,
        help='GRU hidden size'
    )
    parser.add_argument(
        '--layers', type=int, default=2,
        help='Number of stacked GRU layers'
    )
    parser.add_argument(
        '--patience', type=int, default=20,
        help='Early stopping patience'
    )
    parser.add_argument(
        '--stride', type=int, default=1,
        help='Sliding window stride (1 = every sample)'
    )
    parser.add_argument(
        '--val-fraction', type=float, default=0.2,
        help='Fraction of data for validation'
    )

    args = parser.parse_args()

    # ── Device selection ──
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # ── Load data ──
    print(f"Loading data from {args.data} ...")
    raw = load_csv_data(args.data)
    N = len(raw['theta'])
    print(f"  Loaded {N} samples ({N/200:.1f}s at 200Hz)")

    # ── Build datasets ──
    print(f"Building datasets (window={args.window}, stride={args.stride}) ...")
    datasets = build_observer_datasets(
        raw,
        window_size=args.window,
        stride=args.stride,
        val_fraction=args.val_fraction,
    )

    # ── DataLoaders ──
    enc_train_loader = DataLoader(
        datasets['encoder_train'], batch_size=args.batch_size,
        shuffle=True, num_workers=0, pin_memory=(device.type == 'cuda'),
    )
    enc_val_loader = DataLoader(
        datasets['encoder_val'], batch_size=args.batch_size,
        shuffle=False, num_workers=0, pin_memory=(device.type == 'cuda'),
    )
    force_train_loader = DataLoader(
        datasets['force_train'], batch_size=args.batch_size,
        shuffle=True, num_workers=0, pin_memory=(device.type == 'cuda'),
    )
    force_val_loader = DataLoader(
        datasets['force_val'], batch_size=args.batch_size,
        shuffle=False, num_workers=0, pin_memory=(device.type == 'cuda'),
    )

    # ── Create models ──
    encoder_obs = EncoderObserver(
        input_size=3, hidden_size=args.hidden,
        num_layers=args.layers, dropout=0.1,
    )
    force_obs = ForceObserver(
        input_size=3, hidden_size=args.hidden,
        num_layers=args.layers, dropout=0.1,
    )

    # ── Set normalization from training data stats ──
    enc_in_mean, enc_in_std = datasets['norm_encoder_input']
    enc_out_mean, enc_out_std = datasets['norm_encoder_output']
    encoder_obs.set_normalization(
        torch.from_numpy(enc_in_mean).float(),
        torch.from_numpy(enc_in_std).float(),
        torch.from_numpy(enc_out_mean).float(),
        torch.from_numpy(enc_out_std).float(),
    )

    force_in_mean, force_in_std = datasets['norm_force_input']
    force_out_mean, force_out_std = datasets['norm_force_output']
    force_obs.set_normalization(
        torch.from_numpy(force_in_mean).float(),
        torch.from_numpy(force_in_std).float(),
        torch.from_numpy(force_out_mean).float(),
        torch.from_numpy(force_out_std).float(),
    )

    # ── Train Observer A (Encoder) ──
    enc_result = train_observer(
        encoder_obs, enc_train_loader, enc_val_loader,
        device, name="Encoder",
        max_epochs=args.epochs, lr=args.lr, patience=args.patience,
    )

    # ── Train Observer B (Force) ──
    force_result = train_observer(
        force_obs, force_train_loader, force_val_loader,
        device, name="Force",
        max_epochs=args.epochs, lr=args.lr, patience=args.patience,
    )

    # ── Save models ──
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    enc_path = output_dir / 'encoder_observer.pt'
    force_path = output_dir / 'force_observer.pt'

    # Save complete checkpoint (model + config + training info)
    torch.save({
        'model_state_dict': encoder_obs.state_dict(),
        'config': {
            'input_size': 3,
            'hidden_size': args.hidden,
            'num_layers': args.layers,
            'window_size': args.window,
        },
        'best_val_loss': enc_result['best_val_loss'],
        'history': enc_result['history'],
    }, enc_path)

    torch.save({
        'model_state_dict': force_obs.state_dict(),
        'config': {
            'input_size': 3,
            'hidden_size': args.hidden,
            'num_layers': args.layers,
            'window_size': args.window,
        },
        'best_val_loss': force_result['best_val_loss'],
        'history': force_result['history'],
    }, force_path)

    print(f"\n{'='*60}")
    print(f"  Models saved to {output_dir}/")
    print(f"    encoder_observer.pt  (val_loss={enc_result['best_val_loss']:.6f})")
    print(f"    force_observer.pt    (val_loss={force_result['best_val_loss']:.6f})")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()