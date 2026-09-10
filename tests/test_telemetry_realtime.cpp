#include "neurokinetic/telemetry/spsc_ring_buffer.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <thread>

using neurokinetic::telemetry::KinematicTelemetryFrame;
using neurokinetic::telemetry::SpscRingBuffer;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "telemetry realtime test failed: " << message << '\n';
        std::terminate();
    }
}

int main() {
    SpscRingBuffer<KinematicTelemetryFrame, 1024> queue;
    std::atomic<bool> producer_done{false};
    std::atomic<uint64_t> consumed{0};
    std::atomic<uint64_t> last_timestamp{0};
    constexpr uint64_t samples = 100000;

    std::thread consumer([&] {
        KinematicTelemetryFrame frame{};
        for (;;) {
            if (queue.try_pop(frame)) {
                require(frame.timestamp_ns > last_timestamp.load(std::memory_order_relaxed), "data corruption or ordering failure");
                last_timestamp.store(frame.timestamp_ns, std::memory_order_relaxed);
                consumed.fetch_add(1, std::memory_order_relaxed);
            } else if (producer_done.load(std::memory_order_acquire)) {
                break;
            } else {
                std::this_thread::yield();
            }
        }
    });

    std::thread producer([&] {
        for (uint64_t index = 1; index <= samples; ++index) {
            KinematicTelemetryFrame frame{index, 1.0f, 2.0f, 0.5f, 0};
            while (!queue.try_push(frame)) std::this_thread::yield();
        }
        producer_done.store(true, std::memory_order_release);
    });

    producer.join();
    consumer.join();
    require(consumed.load() == samples, "not all samples were consumed");

    SpscRingBuffer<KinematicTelemetryFrame, 4> full_queue;
    KinematicTelemetryFrame frame{1, 1.0f, 0.0f, 0.0f, 0};
    require(full_queue.try_push(frame), "first push failed");
    require(full_queue.try_push(frame), "second push failed");
    require(full_queue.try_push(frame), "third push failed");
    const auto start = std::chrono::steady_clock::now();
    require(!full_queue.try_push(frame), "overflow push did not return false");
    const auto elapsed = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
    require(elapsed < 1000.0, "overflow push blocked for too long");
    require(full_queue.dropped() == 1, "overflow was not counted");

    SpscRingBuffer<KinematicTelemetryFrame, 2> latency_queue;
    const auto latency_start = std::chrono::steady_clock::now();
    for (uint64_t index = 0; index < samples; ++index) {
        require(latency_queue.try_push(frame), "single-thread push failed");
        require(latency_queue.try_pop(frame), "single-thread pop failed");
    }
    const double average_push_pop_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - latency_start).count() / static_cast<double>(samples);
    require(average_push_pop_us < 1.0, "average queue operation exceeded one microsecond");

    std::cout << "telemetry realtime test passed; consumed " << consumed.load()
              << " samples, overflow path " << elapsed << " us, average push/pop "
              << average_push_pop_us << " us\n";
    return 0;
}
