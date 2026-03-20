#!/usr/bin/env python3
"""
logger_node.py
==============

Logger ROS2 per il digital twin dell'esoscheletro.

Funzione:
- sottoscrive i topic diagnostici principali del plant e dei controllori
- accumula l'ultimo valore ricevuto per ciascun blocco
- scrive periodicamente una riga CSV con timestamp locale e tutti i campi

Topic sottoscritti:
  /exo_dynamics/debug         std_msgs/Float64MultiArray
  /exo_dynamics/ff_terms      std_msgs/Float64MultiArray
  /exo_dynamics/model_debug   std_msgs/Float64MultiArray   (se presente)
  /traj_ctrl/debug            std_msgs/Float64MultiArray
  /admittance/debug           std_msgs/Float64MultiArray
  /joint_states               sensor_msgs/JointState       (opzionale, ridondanza)
  /torque                     std_msgs/Float64             (opzionale, ridondanza)

Output:
  CSV con colonne stabili e flush periodico su disco.

Note:
- Se un topic non è ancora arrivato, i campi corrispondenti vengono scritti come NaN.
- /exo_dynamics/model_debug è opzionale: se il topic non esiste, le colonne restano NaN.
"""

from pathlib import Path
import csv
import math
import os
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


CSV_COLUMNS = [
    "t_sec",

    # ---------------- plant debug ----------------
    "theta",
    "theta_dot",
    "theta_ddot",
    "tau_m",
    "denom",
    "proj",
    "g_proj",
    "closure_norm",
    "solver_success",
    "solver_nfev",
    "hit_bounds",
    "at_limit",
    "tau_ext_theta",
    "tau_pass_theta",
    "reaction_theta",

    # ---------------- ff terms ----------------
    "ff_M_eff",
    "ff_proj",
    "ff_g_proj",
    "ff_tau_pass_theta",

    # ---------------- model debug ----------------
    "mdl_theta",
    "mdl_theta_dot",
    "mdl_theta_ddot",
    "mdl_tau_m",
    "mdl_tau_ext_theta",
    "mdl_tau_pass_theta",
    "mdl_M_eff",
    "mdl_proj",
    "mdl_g_proj",
    "mdl_tau_fric",
    "mdl_tau_damp",
    "mdl_dyn_num",
    "mdl_dyn_residual",
    "mdl_tau_model",
    "mdl_tau_model_error",

    # ---------------- trajectory controller ----------------
    "theta_ref",
    "ctrl_theta",
    "ctrl_e_theta",
    "theta_dot_ref",
    "ctrl_theta_dot",
    "ctrl_e_theta_dot",
    "ctrl_tau_pd",
    "ctrl_tau_ff",
    "ctrl_tau_raw",
    "ctrl_tau_out",
    "ctrl_M_eff",
    "ctrl_g_proj",
    "ctrl_tau_pass_theta",

    # ---------------- admittance ----------------
    "adm_theta_v",
    "adm_theta_dot_v",
    "adm_theta_ddot_v",
    "adm_tau_ext_theta_raw",
    "adm_tau_in",
    "adm_tau_spring",
    "adm_tau_damper",
    "adm_theta_eq",
    "adm_M",
    "adm_D",
    "adm_K",

    # ---------------- opzionali / ridondanza ----------------
    "js_theta",
    "js_theta_dot",
    "tau_cmd",
]


def nan_row() -> Dict[str, float]:
    return {c: float("nan") for c in CSV_COLUMNS}


class ExoLogger(Node):

    def __init__(self) -> None:
        super().__init__("exo_logger")

        # ------------------------------------------------
        # Parametri
        # ------------------------------------------------
        self.declare_parameter("output_dir", str(Path.home() / "exo_logs"))
        self.declare_parameter("file_name", "exo_log.csv")
        self.declare_parameter("log_rate", 200.0)
        self.declare_parameter("flush_every_n_rows", 20)
        self.declare_parameter("joint_name", "rev_crank")
        self.declare_parameter("overwrite", True)

        output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser().resolve()
        file_name = str(self.get_parameter("file_name").value)
        self.log_rate = float(self.get_parameter("log_rate").value)
        self.flush_every_n_rows = int(self.get_parameter("flush_every_n_rows").value)
        self.joint_name = str(self.get_parameter("joint_name").value)
        overwrite = bool(self.get_parameter("overwrite").value)

        output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = output_dir / file_name

        if self.csv_path.exists() and not overwrite:
            stem = self.csv_path.stem
            suffix = self.csv_path.suffix
            k = 1
            while True:
                candidate = self.csv_path.parent / f"{stem}_{k}{suffix}"
                if not candidate.exists():
                    self.csv_path = candidate
                    break
                k += 1

        # ------------------------------------------------
        # Stato interno
        # ------------------------------------------------
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.rows_written = 0

        self.last: Dict[str, Optional[Dict[str, float]]] = {
            "exo_debug": None,
            "ff_terms": None,
            "model_debug": None,
            "traj_debug": None,
            "adm_debug": None,
            "joint_state": None,
            "torque": None,
        }

        # ------------------------------------------------
        # CSV
        # ------------------------------------------------
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_COLUMNS)
        self.writer.writeheader()
        self.csv_file.flush()

        # ------------------------------------------------
        # Subscribers
        # ------------------------------------------------
        self.sub_exo_debug = self.create_subscription(
            Float64MultiArray,
            "/exo_dynamics/debug",
            self.exo_debug_cb,
            50,
        )

        self.sub_ff_terms = self.create_subscription(
            Float64MultiArray,
            "/exo_dynamics/ff_terms",
            self.ff_terms_cb,
            50,
        )

        self.sub_model_debug = self.create_subscription(
            Float64MultiArray,
            "/exo_dynamics/model_debug",
            self.model_debug_cb,
            50,
        )

        self.sub_traj_debug = self.create_subscription(
            Float64MultiArray,
            "/traj_ctrl/debug",
            self.traj_debug_cb,
            50,
        )

        self.sub_adm_debug = self.create_subscription(
            Float64MultiArray,
            "/admittance/debug",
            self.adm_debug_cb,
            50,
        )

        self.sub_js = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_cb,
            50,
        )

        self.sub_tau = self.create_subscription(
            Float64,
            "/torque",
            self.torque_cb,
            50,
        )

        # ------------------------------------------------
        # Timer di scrittura
        # ------------------------------------------------
        dt = 1.0 / max(self.log_rate, 1e-6)
        self.timer = self.create_timer(dt, self.write_row)

        self.get_logger().info(
            f"ExoLogger avviato | csv={self.csv_path} | log_rate={self.log_rate:.1f} Hz"
        )

    # ====================================================
    # CALLBACKS
    # ====================================================

    def exo_debug_cb(self, msg: Float64MultiArray) -> None:
        """
        Layout /exo_dynamics/debug:
          [0]  theta
          [1]  theta_dot
          [2]  theta_ddot
          [3]  tau_m
          [4]  denom_total
          [5]  proj
          [6]  gproj
          [7]  closure_norm
          [8]  solver_success
          [9]  solver_nfev
          [10] hit_bounds
          [11] at_limit
          [12] tau_ext_theta
          [13] tau_pass_theta
          [14] reaction_theta
        """
        d = list(msg.data)
        self.last["exo_debug"] = {
            "theta": d[0] if len(d) > 0 else math.nan,
            "theta_dot": d[1] if len(d) > 1 else math.nan,
            "theta_ddot": d[2] if len(d) > 2 else math.nan,
            "tau_m": d[3] if len(d) > 3 else math.nan,
            "denom": d[4] if len(d) > 4 else math.nan,
            "proj": d[5] if len(d) > 5 else math.nan,
            "g_proj": d[6] if len(d) > 6 else math.nan,
            "closure_norm": d[7] if len(d) > 7 else math.nan,
            "solver_success": d[8] if len(d) > 8 else math.nan,
            "solver_nfev": d[9] if len(d) > 9 else math.nan,
            "hit_bounds": d[10] if len(d) > 10 else math.nan,
            "at_limit": d[11] if len(d) > 11 else math.nan,
            "tau_ext_theta": d[12] if len(d) > 12 else math.nan,
            "tau_pass_theta": d[13] if len(d) > 13 else math.nan,
            "reaction_theta": d[14] if len(d) > 14 else math.nan,
        }

    def ff_terms_cb(self, msg: Float64MultiArray) -> None:
        """
        Layout /exo_dynamics/ff_terms:
          [0] M_eff
          [1] proj
          [2] g_proj
          [3] tau_pass_theta
        """
        d = list(msg.data)
        self.last["ff_terms"] = {
            "ff_M_eff": d[0] if len(d) > 0 else math.nan,
            "ff_proj": d[1] if len(d) > 1 else math.nan,
            "ff_g_proj": d[2] if len(d) > 2 else math.nan,
            "ff_tau_pass_theta": d[3] if len(d) > 3 else math.nan,
        }

    def model_debug_cb(self, msg: Float64MultiArray) -> None:
        """
        Layout /exo_dynamics/model_debug:
          [0]  theta
          [1]  theta_dot
          [2]  theta_ddot
          [3]  tau_m
          [4]  tau_ext_theta
          [5]  tau_pass_theta
          [6]  M_eff
          [7]  proj
          [8]  g_proj
          [9]  tau_fric
          [10] tau_damp
          [11] dyn_num
          [12] dyn_residual
          [13] tau_model
          [14] tau_model_error
        """
        d = list(msg.data)
        self.last["model_debug"] = {
            "mdl_theta": d[0] if len(d) > 0 else math.nan,
            "mdl_theta_dot": d[1] if len(d) > 1 else math.nan,
            "mdl_theta_ddot": d[2] if len(d) > 2 else math.nan,
            "mdl_tau_m": d[3] if len(d) > 3 else math.nan,
            "mdl_tau_ext_theta": d[4] if len(d) > 4 else math.nan,
            "mdl_tau_pass_theta": d[5] if len(d) > 5 else math.nan,
            "mdl_M_eff": d[6] if len(d) > 6 else math.nan,
            "mdl_proj": d[7] if len(d) > 7 else math.nan,
            "mdl_g_proj": d[8] if len(d) > 8 else math.nan,
            "mdl_tau_fric": d[9] if len(d) > 9 else math.nan,
            "mdl_tau_damp": d[10] if len(d) > 10 else math.nan,
            "mdl_dyn_num": d[11] if len(d) > 11 else math.nan,
            "mdl_dyn_residual": d[12] if len(d) > 12 else math.nan,
            "mdl_tau_model": d[13] if len(d) > 13 else math.nan,
            "mdl_tau_model_error": d[14] if len(d) > 14 else math.nan,
        }

    def traj_debug_cb(self, msg: Float64MultiArray) -> None:
        """
        Layout /traj_ctrl/debug:
          [0] theta_ref
          [1] theta
          [2] e_theta
          [3] theta_dot_ref
          [4] theta_dot
          [5] e_theta_dot
          [6] tau_pd
          [7] tau_ff
          [8] tau_raw
          [9] tau_out
          [10] M_eff
          [11] g_proj
          [12] tau_pass_theta
        """
        d = list(msg.data)
        self.last["traj_debug"] = {
            "theta_ref": d[0] if len(d) > 0 else math.nan,
            "ctrl_theta": d[1] if len(d) > 1 else math.nan,
            "ctrl_e_theta": d[2] if len(d) > 2 else math.nan,
            "theta_dot_ref": d[3] if len(d) > 3 else math.nan,
            "ctrl_theta_dot": d[4] if len(d) > 4 else math.nan,
            "ctrl_e_theta_dot": d[5] if len(d) > 5 else math.nan,
            "ctrl_tau_pd": d[6] if len(d) > 6 else math.nan,
            "ctrl_tau_ff": d[7] if len(d) > 7 else math.nan,
            "ctrl_tau_raw": d[8] if len(d) > 8 else math.nan,
            "ctrl_tau_out": d[9] if len(d) > 9 else math.nan,
            "ctrl_M_eff": d[10] if len(d) > 10 else math.nan,
            "ctrl_g_proj": d[11] if len(d) > 11 else math.nan,
            "ctrl_tau_pass_theta": d[12] if len(d) > 12 else math.nan,
        }

    def adm_debug_cb(self, msg: Float64MultiArray) -> None:
        """
        Layout /admittance/debug:
          [0]  theta_v
          [1]  theta_dot_v
          [2]  theta_ddot_v
          [3]  tau_ext_theta_raw
          [4]  tau_in
          [5]  tau_spring
          [6]  tau_damper
          [7]  theta_eq
          [8]  M
          [9]  D
          [10] K
        """
        d = list(msg.data)
        self.last["adm_debug"] = {
            "adm_theta_v": d[0] if len(d) > 0 else math.nan,
            "adm_theta_dot_v": d[1] if len(d) > 1 else math.nan,
            "adm_theta_ddot_v": d[2] if len(d) > 2 else math.nan,
            "adm_tau_ext_theta_raw": d[3] if len(d) > 3 else math.nan,
            "adm_tau_in": d[4] if len(d) > 4 else math.nan,
            "adm_tau_spring": d[5] if len(d) > 5 else math.nan,
            "adm_tau_damper": d[6] if len(d) > 6 else math.nan,
            "adm_theta_eq": d[7] if len(d) > 7 else math.nan,
            "adm_M": d[8] if len(d) > 8 else math.nan,
            "adm_D": d[9] if len(d) > 9 else math.nan,
            "adm_K": d[10] if len(d) > 10 else math.nan,
        }

    def joint_states_cb(self, msg: JointState) -> None:
        try:
            idx = list(msg.name).index(self.joint_name)
        except ValueError:
            return

        js_theta = float(msg.position[idx]) if idx < len(msg.position) else math.nan
        js_theta_dot = float(msg.velocity[idx]) if idx < len(msg.velocity) else math.nan

        self.last["joint_state"] = {
            "js_theta": js_theta,
            "js_theta_dot": js_theta_dot,
        }

    def torque_cb(self, msg: Float64) -> None:
        self.last["torque"] = {
            "tau_cmd": float(msg.data),
        }

    # ====================================================
    # SCRITTURA CSV
    # ====================================================

    def write_row(self) -> None:
        row = nan_row()
        now = self.get_clock().now().nanoseconds * 1e-9
        row["t_sec"] = now - self.t0

        for block in self.last.values():
            if block is not None:
                row.update(block)

        self.writer.writerow(row)
        self.rows_written += 1

        if self.rows_written % max(self.flush_every_n_rows, 1) == 0:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())

    # ====================================================
    # SHUTDOWN
    # ====================================================

    def destroy_node(self):
        try:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()
        except Exception:
            pass
        self.get_logger().info(f"CSV salvato in: {self.csv_path}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExoLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()