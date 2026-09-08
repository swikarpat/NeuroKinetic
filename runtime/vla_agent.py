import os
import sys
from pathlib import Path

# Ensure repository root is on sys.path regardless of execution context
ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

import time
import math
import numpy as np
from runtime.shm_client import NeuroKineticClient, DOF
from runtime.fno_surrogate import fno_twin

class VLACognitivePlanner:
    def __init__(self):
        self.client = NeuroKineticClient()
        print("✓ Connected to 500 Hz C++23 Safety Daemon via POSIX SHM")

    def plan_trajectory_chunk(self, start_pos: list[float], target_pos: list[float], duration_sec: float = 2.0, steps: int = 50):
        """
        Generates minimum-jerk cubic Hermite trajectory chunk.
        """
        t_arr = np.linspace(0.0, 1.0, steps)
        s_curve = 3.0 * (t_arr ** 2) - 2.0 * (t_arr ** 3)
        
        trajectory = []
        for s in s_curve:
            interp = [start_pos[j] + s * (target_pos[j] - start_pos[j]) for j in range(DOF)]
            trajectory.append(interp)
        return trajectory

    def execute_task(self, prompt: str, target_pos: list[float]):
        print(f"\n========================================================")
        print(f"  Task Ingestion: '{prompt}'")
        print(f"========================================================")

        # 1. Read current robot telemetry from C++ daemon
        initial_state = self.client.read_telemetry()
        current_pos = initial_state["positions"]
        print(f"[Initial Joint State] Pos[5]: {current_pos[5]:.3f} rad | Margin: {initial_state['barrier_margin']:.3f} m")

        # 2. VLA Action Chunk Generation (50 Hz setpoint density over 1.5 seconds)
        trajectory_chunk = self.plan_trajectory_chunk(current_pos, target_pos, duration_sec=1.5, steps=30)
        print(f"✓ VLA Policy generated {len(trajectory_chunk)} trajectory setpoints")

        # 3. Stream trajectory setpoints through physics twin and down to SHM
        print("\nStreaming setpoints to 500 Hz safety daemon with live FNO physics evaluation:")
        
        for idx, setpoint in enumerate(trajectory_chunk):
            t_start = time.perf_counter()

            # Dynamic PD feedforward torque calculation
            nominal_torques = [0.0] * DOF
            for j in range(DOF):
                pos_error = setpoint[j] - current_pos[j]
                nominal_torques[j] = float(np.clip(pos_error * 45.0, -80.0, 80.0))

            # Trigger synthetic hazard on Joint 5 midway through to demonstrate CBF override
            if idx > 12:
                nominal_torques[5] = 42.0  # Hazard directed at safety envelope

            # Dispatch via Zero-Copy POSIX Shared Memory
            self.client.dispatch_vla_torque(nominal_torques)

            # Read back verified applied state from C++ daemon
            live_telem = self.client.read_telemetry()
            current_pos = live_telem["positions"]

            # Evaluate Neural Physics Twin (Thermal & Structural Stress)
            physics = fno_twin.evaluate_physics_twin(
                current_pos, 
                live_telem["velocities"], 
                live_telem["torques"]
            )

            if idx % 3 == 0:
                print(f" • Step {idx:02d} | Commanded T[5]: {nominal_torques[5]:5.1f} Nm | "
                      f"Safe Applied: {live_telem['torques'][5]:6.2f} Nm | "
                      f"Clamped: {str(live_telem['clamped']):<5} | "
                      f"Max Stress: {physics['max_stress_mpa']:5.1f} MPa | "
                      f"Peak Temp: {physics['max_temp_c']:4.1f} °C (FNO: {physics['fno_eval_time_ms']} ms)")

            # Maintain 50 Hz pacing (20 ms loop)
            elapsed = time.perf_counter() - t_start
            if elapsed < 0.020:
                time.sleep(0.020 - elapsed)

        final_telem = self.client.read_telemetry()
        print(f"\n[Task Complete] Final Pos[5]: {final_telem['positions'][5]:.3f} rad | Final Barrier Margin: {final_telem['barrier_margin']:.3f} m")

if __name__ == "__main__":
    planner = VLACognitivePlanner()
    goal_joints = [0.25, 0.40, -0.10, -0.90, 0.15, 2.88, 0.05]
    planner.execute_task(
        prompt="Transfer hot payload to Pallet C and avoid inspection zone obstacle",
        target_pos=goal_joints
    )
