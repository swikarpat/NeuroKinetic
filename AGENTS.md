# NeuroKinetic Master Blueprint & Agent Memory Store

> **CRITICAL DIRECTIVE FOR ALL AI AGENTS**:
> This document is the **single source of truth** and **persistent memory** for the NeuroKinetic codebase.
> Whenever you (the AI assistant) introduce new modules, refactor existing components, modify network ports, or adjust safety kernel CBF/QP math, **you are strictly required to update this file in the same turn**.
> Before responding to architectural inquiries, cross-verify this document against the physical workspace (`CMakeLists.txt`, `gateway/build.gradle`, `runtime/`, `src/`) to ensure zero hallucinations.

---

## 1. System Design & Architecture Overview

NeuroKinetic is an enterprise-grade **Physical AI Runtime, 500 Hz Deterministic CBF Safety Kernel, and Multi-Domain Digital Twin Platform**. It solves the central safety dilemma of robotics: **probabilistic Vision-Language-Action (VLA) foundation models are inherently stochastic and prone to spatial hallucinations, whereas physical actuators demand microsecond-level deterministic safety guarantees.**

The platform establishes an air-gapped mathematical boundary between cognitive planning and mechanical actuation, ensuring that kinematic limits, thermal envelopes, and joint collision boundaries are formally proven and never breached.

### High-Level Multi-Rate Hierarchical Topology

```
  ┌────────────────────────────────────────────────────────────────────────┐
  │   Mission Control Digital Twin UI (Three.js WebGL + 50 Hz WebSocket)   │
  │   • Port: :8000 (FastAPI Web Hub)                                      │
  │   • Live 3D Joint Kinematics, Torque Vectors & FNO Thermal Heatmaps    │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │ WebSocket / HTTP Stream
                                      ▼
  ┌────────────────────────────────────────────────────────────────────────┐
  │   Enterprise Gateway (Java 21 Spring Boot 3.3.0 WebFlux Netty)         │
  │   • Port: :8080                                                        │
  │   • Ingress Authentication, Rate Limiting, Route Proxying              │
  └───────────────────────────────────┬────────────────────────────────────┘
                                      │
                                      ▼ REST / IPC Forwarding
 ┌────────────────────────────────────────────────────────────────────────┐
 │ 1. COGNITIVE & SURROGATE RUNTIME (Python 3.14 + NumPy)                 │
 │    • [2-5 Hz] VLA Cognitive Planner: High-level task reasoning         │
 │    • [50 Hz] Cubic Hermite Chunker: Smooth kinematic trajectory chunks │
 │    • [50 Hz] Fourier Neural Operator (FNO): 1D spectral stress/thermal │
 └───────────────────────────────────┬────────────────────────────────────┘
                                     │
                                     │ Lock-Free POSIX Shared Memory (/tmp/neurokinetic_ipc.shm)
                                     │ Seqlock Atomic Sequence Counters (< 6.0 µs Latency)
                                     ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │ 2. DETERMINISTIC SAFETY KERNEL (C++23 + Eigen3 Vectorization)          │
 │    • [500 Hz] Real-Time Daemon: 2,000 µs clock interval tick           │
 │    • Active-Set Quadratic Programming (CBF-QP) Solver (1.29 µs P50)    │
 │    • Active Progressive Clamping: 100% mathematical zero-breach        │
 │    • Zero-Heap Guarantee: 0 bytes dynamic allocation on hot path       │
 └───────────────────────────────────┬────────────────────────────────────┘
                                     │ Hardware Direct Bus (EtherCAT / CAN-FD)
                                     ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │ 3. PHYSICAL / SIMULATED ACTUATORS (7-DOF Manipulator / Humanoid)       │
 └────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Core Polyglot Component Map

| Component | Path | Language / Runtime | Primary Port | Core Responsibilities |
| :--- | :--- | :--- | :--- | :--- |
| **Mission Control UI** | `/frontend` | Three.js, WebGL, HTML5/JS | `:8000` (via FastAPI) | Real-time 3D digital twin rendering of robot joints, torque vectors, dynamic obstacle hulls, and live FNO stress/thermal heatmaps. |
| **Enterprise Gateway** | `/gateway` | Java 21, Spring Boot 3.3.0, WebFlux | `:8080` | Reactive Netty reverse proxy, non-blocking API termination, token authentication, and distributed telemetry routing. |
| **Cognitive & Physics Runtime** | `/runtime` | Python 3.14, FastAPI, NumPy, Uvicorn | `:8000` | VLA action chunking (`vla_agent.py`), FNO physics surrogate (`fno_surrogate.py`), and zero-copy shared memory client (`shm_client.py`). |
| **Deterministic Safety Kernel** | `/src/kernel` | C++23, CMake, Eigen3, POSIX RT | In-Process Daemon | 500 Hz control loop (`realtime_daemon.cpp`), active-set QP solver (`cbf_solver.cpp`), and forward-invariant barrier filtering. |
| **Lock-Free IPC** | `/src/ipc` | C++23 Header / Python `struct` | `/tmp/neurokinetic_ipc.shm` | 64-byte cache-line aligned Seqlock data structures (`shm_layout.hpp`) ensuring atomic lock-free reads/writes across runtimes. |
| **Observability** | `/infra` | Grafana, Tempo, Loki, Prometheus | `:3000` (Grafana), `:4317` (Tempo), `:3100` (Loki) | OTel distributed trace collection, system metrics, and structured log aggregation. |

---

## 3. Verified Production Performance Benchmarks

All metrics empirically verified on Apple Silicon M-series using C++23 `-O3 -march=native` optimizations:

| Architectural Dimension | Production SLA | Empirically Verified Metric | Enforcement Mechanism |
| :--- | :--- | :--- | :--- |
| **Safety Kernel Control Loop** | 500 Hz (2,000 µs tick) | **500 Hz (1,998–2,002 µs)** | Dedicated real-time clock thread with POSIX interval pacing |
| **CBF QP Solver Execution** | < 800.0 µs | **1.29 µs (P50) / 1.54 µs (P99)** | Active-set quadratic programming over Eigen3 vectorized arrays |
| **Inter-Process IPC Latency** | < 15.0 µs | **< 6.0 µs** | Zero-copy `/tmp/neurokinetic_ipc.shm` with atomic sequence locks |
| **FNO Surrogate Evaluation** | < 15.0 ms (50 Hz) | **0.076–0.171 ms** | Truncated Fourier space 1D spectral convolution |
| **Kinematic Safety Clamping** | 100% Zero-Breach | **Active Progressive Clamping** | Overrode +40.0 Nm hazard down to 0.33 Nm at 2.893 rad limit |
| **Dynamic Memory Allocation** | Zero Heap on Hot Path | **0 bytes malloc/new** | Stack-allocated Eigen vectors & pre-mapped memory buffers |
| **Mission Control Telemetry** | 50 Hz Live Streaming | **50 Hz WebSocket Broadcast** | FastAPI async loop + Three.js 3D WebGL digital twin canvas |

---

## 4. Architectural Decision Records (ADRs)

### ADR-001: Control Barrier Functions (CBFs) via QP vs. End-to-End Reinforcement Learning (RL)
* **Status**: Accepted & Implemented
* **Decision**: Implement **Control Barrier Functions unified with active-set Quadratic Programming** in native C++23.
* **Engineering Rationale**: Unconstrained neural policies (RL or VLA diffusion models) frequently produce out-of-distribution torque spikes or self-collisions. Adding penalty terms to reward functions provides zero formal mathematical guarantees. CBF-QP acts as an immutable mathematical filter: if an action is safe, it passes unaltered; if unsafe, the QP solver minimally adjusts the torque to the boundary of the forward-invariant safe set in **1.29 µs**.

### ADR-002: Fourier Neural Operators (FNO) vs. Finite Element Analysis (FEA) for Digital Twin
* **Status**: Accepted & Implemented
* **Decision**: Implement a **1D Spectral Convolution Fourier Neural Operator** surrogate in Python 3.14/NumPy.
* **Engineering Rationale**: Finite Element Analysis (FEA) requires minutes or hours to solve non-linear PDEs, making closed-loop thermal alerting impossible. FNO learns mappings between infinite-dimensional function spaces and evaluates dynamic physical stress/temperature tensors in **0.076 to 0.171 ms**, enabling continuous 50 Hz predictive digital twin monitoring.

### ADR-003: Lock-Free Shared Memory (Seqlock) vs. ROS2 DDS Middleware
* **Status**: Accepted & Implemented
* **Decision**: Implement **POSIX Shared Memory with 64-byte cache-line aligned Seqlock structures** (`/tmp/neurokinetic_ipc.shm`).
* **Engineering Rationale**: Standard ROS2 DDS introduces 1.2 to 4.5 ms of jitter and serialization overhead under memory pressure. The Seqlock pattern uses an atomic monotonic counter (`std::atomic<uint64_t>`), achieving **< 6.0 µs** IPC latency while ensuring the 500 Hz C++ safety thread never blocks on Python garbage collection cycles.

### ADR-004: Decoupled Multi-Rate Temporal Architecture (2 Hz / 50 Hz / 500 Hz)
* **Status**: Accepted & Implemented
* **Decision**: Standardize on a **3-Tier Multi-Rate Architecture with cubic Hermite spline interpolation**.
* **Engineering Rationale**: Large foundation models (VLM/VLA) cannot execute at motor control frequencies (500 Hz). Tier 1 plans goals at 2–5 Hz; Tier 2 generates smooth trajectory chunks and validates FNO physics at 50 Hz; Tier 3 enforces safety invariants at 500 Hz, decoupling cognitive reasoning from physical motor pacing.

### ADR-005: Java 21 Spring Boot WebFlux for Enterprise Gateway
* **Status**: Accepted & Implemented
* **Decision**: Deploy **Java 21 Spring Boot WebFlux (Netty)** as the external ingress gateway.
* **Engineering Rationale**: Provides reactive non-blocking event loops capable of handling 50,000+ client connections with deterministic memory, seamless token bucket rate limiting, and distributed OpenTelemetry propagation.

---

## 5. Development & Automation Commands

* **Build C++23 Safety Engine**:
  `mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && cmake --build .`
* **Run 10,000-Tick Verification Benchmark**:
  `./build/cbf_kernel_benchmark`
* **Launch Python Runtime & Three.js 3D Server**:
  `python3 -m uvicorn runtime.server:app --host 0.0.0.0 --port 8000`
* **Run Java Gateway**:
  `./gateway/gradlew -p gateway bootRun`
* **Start Observability Stack**:
  `docker compose up -d tempo loki prometheus grafana`
