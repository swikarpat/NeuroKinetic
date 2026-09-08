import mmap
import struct
import time
import sys
import os

SHM_PATH = "/tmp/neurokinetic_ipc.shm"
DOF = 7

# Exact 64-byte aligned offsets matching C++23 alignas(64) struct packing:
# Offset 0..15   : Heartbeats (2x uint64) + 48 bytes padding to alignas(64)
# Offset 64..191 : TelemetrySlot (128 bytes: 2x uint64, 21x float, 1x float, 2x uint32, 24x padding)
# Offset 192..319: CommandSlot   (128 bytes: 2x uint64, 14x float, 1x uint32, 52x padding)
TELEMETRY_OFFSET = 64
TELEMETRY_FORMAT = "=QQ7f7f7ffII24x"   # Exactly 128 bytes
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)

COMMAND_OFFSET = 192
COMMAND_FORMAT = "=QQ7f7fI52x"        # Exactly 128 bytes
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)

class NeuroKineticClient:
    def __init__(self):
        if not os.path.exists(SHM_PATH):
            raise FileNotFoundError(f"Shared memory file '{SHM_PATH}' not found. Start ./neurokinetic_daemon first.")
        
        self.shm_file = open(SHM_PATH, "r+b")
        self.buf = mmap.mmap(self.shm_file.fileno(), 0)
        self.cmd_seq = 0

    def read_telemetry(self, timeout_sec: float = 1.0) -> dict:
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            # Atomic seqlock check
            seq1 = struct.unpack_from("=Q", self.buf, TELEMETRY_OFFSET)[0]
            if seq1 % 2 != 0 or seq1 == 0:
                time.sleep(0.0002)
                continue

            raw = self.buf[TELEMETRY_OFFSET:TELEMETRY_OFFSET + TELEMETRY_SIZE]
            data = struct.unpack(TELEMETRY_FORMAT, raw)

            seq2 = struct.unpack_from("=Q", self.buf, TELEMETRY_OFFSET)[0]
            if seq1 == seq2:
                return {
                    "sequence": data[0],
                    "timestamp_ns": data[1],
                    "positions": list(data[2:9]),
                    "velocities": list(data[9:16]),
                    "torques": list(data[16:23]),
                    "barrier_margin": data[23],
                    "clamped": bool(data[24]),
                    "error_code": data[25]
                }
            time.sleep(0.0002)

        raise TimeoutError("Timed out reading consistent telemetry from C++ daemon.")

    def dispatch_vla_torque(self, nominal_torques: list[float], flags: int = 1):
        assert len(nominal_torques) == DOF
        self.cmd_seq += 1

        # 1. Announce write in-flight (odd sequence)
        struct.pack_into("=Q", self.buf, COMMAND_OFFSET, self.cmd_seq)

        # 2. Write payload
        ts = time.time_ns()
        payload = struct.pack(
            COMMAND_FORMAT,
            self.cmd_seq + 1,
            ts,
            *([0.0] * DOF),
            *nominal_torques,
            flags
        )
        self.buf[COMMAND_OFFSET:COMMAND_OFFSET + COMMAND_SIZE] = payload

        # 3. Mark write complete (even sequence)
        self.cmd_seq += 1
        struct.pack_into("=Q", self.buf, COMMAND_OFFSET, self.cmd_seq)

if __name__ == "__main__":
    print("========================================================")
    print("  NeuroKinetic Python 3.14 Zero-Copy SHM Client Online  ")
    print("========================================================")
    
    client = NeuroKineticClient()
    print("✓ Successfully connected to /tmp/neurokinetic_ipc.shm\n")

    # Read clean baseline state before dispatching hazardous command
    baseline = client.read_telemetry()
    print(f"[Baseline State] Tick: {baseline['sequence']} | Joint 5 Position: {baseline['positions'][5]:.3f} rad | Applied: {baseline['torques'][5]:.2f} Nm")

    # Dispatch hazardous torque (+40.0 Nm towards boundary limit)
    print("\n==> Dispatching hazardous VLA policy torque (+40.0 Nm on Joint 5)...")
    hazardous = [0.0, 0.0, 0.0, 0.0, 0.0, 40.0, 0.0]
    client.dispatch_vla_torque(hazardous)

    print("\n[Sampling Active 500 Hz Safe Overrides]")
    for i in range(10):
        time.sleep(0.01)  # Sample every 10 ms (every 5 daemon ticks)
        telem = client.read_telemetry()
        print(f"  • Sample {i+1:02d} | Tick: {telem['sequence']} | Pos[5]: {telem['positions'][5]:.3f} rad | Safe Applied: {telem['torques'][5]:6.2f} Nm | Margin: {telem['barrier_margin']:6.3f} m | Clamped: {telem['clamped']}")
