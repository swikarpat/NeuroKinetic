#pragma once

#include "neurokinetic/telemetry/spsc_ring_buffer.hpp"
#include <atomic>
#include <cstdint>
#include <thread>

namespace neurokinetic::telemetry {

class TelemetryWorker {
public:
    explicit TelemetryWorker(uint16_t metrics_port = 9090) noexcept;
    ~TelemetryWorker();

    TelemetryWorker(const TelemetryWorker&) = delete;
    TelemetryWorker& operator=(const TelemetryWorker&) = delete;

    void start();
    void stop() noexcept;
    bool try_push(const KinematicTelemetryFrame& frame) noexcept;

private:
    void drain_loop() noexcept;
    void metrics_loop() noexcept;
    void record(const KinematicTelemetryFrame& frame) noexcept;
    void export_safety_event(const KinematicTelemetryFrame& frame) noexcept;

    SpscRingBuffer<KinematicTelemetryFrame, 4096> queue_;
    std::atomic<bool> running_{false};
    uint16_t metrics_port_;
    std::thread worker_thread_;
    std::thread metrics_thread_;

    alignas(64) std::atomic<uint64_t> loop_samples_{0};
    std::atomic<uint64_t> loop_sum_us_{0};
    std::atomic<uint64_t> loop_buckets_[6]{};
    std::atomic<uint32_t> condition_number_bits_{0};
    std::atomic<uint32_t> torque_ratio_bits_{0};
    std::atomic<uint64_t> safety_violations_[3]{};
    std::atomic<uint64_t> exported_events_{0};
};

} // namespace neurokinetic::telemetry
