#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace neurokinetic::telemetry {

struct KinematicTelemetryFrame {
    uint64_t timestamp_ns{};
    float loop_duration_us{};
    float condition_number{};
    float max_torque_saturation_ratio{};
    uint32_t safety_flags{};
};

static_assert(std::is_trivially_copyable_v<KinematicTelemetryFrame>);

template <typename T, std::size_t Capacity>
class alignas(64) SpscRingBuffer {
    static_assert(Capacity > 1);
    static_assert(std::is_trivially_copyable_v<T>);

public:
    bool try_push(const T& value) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = increment(head);
        if (next == tail_.load(std::memory_order_acquire)) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        slots_[head] = value;
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool try_pop(T& value) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        value = slots_[tail];
        tail_.store(increment(tail), std::memory_order_release);
        return true;
    }

    uint64_t dropped() const noexcept {
        return dropped_.load(std::memory_order_relaxed);
    }

private:
    static constexpr std::size_t increment(std::size_t index) noexcept {
        return (index + 1) % Capacity;
    }

    alignas(64) std::array<T, Capacity> slots_{};
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
    alignas(64) std::atomic<uint64_t> dropped_{0};
};

} // namespace neurokinetic::telemetry
