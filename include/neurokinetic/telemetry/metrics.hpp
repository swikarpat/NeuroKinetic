#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <string>

namespace neurokinetic::telemetry {

class Metrics {
public:
    void observe_loop_execution_duration_us(double value) noexcept;
    void observe_kinematics_solve_duration_us(double value) noexcept;
    void increment_deadline_misses() noexcept;
    void increment_emergency_stops() noexcept;
    std::string prometheus_text() const;

private:
    static constexpr std::size_t bucket_count = 6;
    void observe(std::array<std::atomic<uint64_t>, bucket_count>& buckets, std::atomic<uint64_t>& sum, double value) noexcept;

    std::array<std::atomic<uint64_t>, bucket_count> loop_buckets_{};
    std::array<std::atomic<uint64_t>, bucket_count> solve_buckets_{};
    std::atomic<uint64_t> loop_sum_milli_us_{0};
    std::atomic<uint64_t> solve_sum_milli_us_{0};
    std::atomic<uint64_t> deadline_misses_{0};
    std::atomic<uint64_t> emergency_stops_{0};
};

} // namespace neurokinetic::telemetry
