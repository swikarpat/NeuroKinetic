#include "storage/flight_recorder.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>

namespace neurokinetic::storage {

FlightRecorder::FlightRecorder(const std::string& db_path) : db_path_(db_path) {
    rocksdb::DBOptions db_options;
    db_options.create_if_missing = true;
    db_options.create_missing_column_families = true;
    db_options.max_background_jobs = 2;

    write_options_fast_.sync = false;
    write_options_fast_.disableWAL = false;

    std::vector<rocksdb::ColumnFamilyDescriptor> column_families;
    column_families.emplace_back(rocksdb::kDefaultColumnFamilyName, rocksdb::ColumnFamilyOptions());
    column_families.emplace_back("cf_cbf_interventions", rocksdb::ColumnFamilyOptions());
    column_families.emplace_back("cf_flight_telemetry", rocksdb::ColumnFamilyOptions());

    std::vector<rocksdb::ColumnFamilyHandle*> handles;
    rocksdb::Status status = rocksdb::DB::Open(db_options, db_path_, column_families, &handles, &db_);

    if (!status.ok()) {
        std::cerr << "[FlightRecorder Error] Failed to open RocksDB: " << status.ToString() << "\n";
        return;
    }

    if (handles.size() >= 3) {
        cf_interventions_ = handles[1];
        cf_telemetry_ = handles[2];
        delete handles[0]; // Drop default CF handle reference
    }
}

FlightRecorder::~FlightRecorder() {
    if (cf_interventions_) {
        delete cf_interventions_;
        cf_interventions_ = nullptr;
    }
    if (cf_telemetry_) {
        delete cf_telemetry_;
        cf_telemetry_ = nullptr;
    }
    db_.reset();
}

bool FlightRecorder::log_cbf_intervention(const InterventionRecord& record) noexcept {
    if (!db_ || !cf_interventions_) return false;

    std::ostringstream key_stream;
    key_stream << "INTERVENE-" << std::setw(20) << std::setfill('0') << record.timestamp_ns;
    std::string key = key_stream.str();

    std::string payload;
    payload.resize(sizeof(InterventionRecord));
    std::memcpy(payload.data(), &record, sizeof(InterventionRecord));

    rocksdb::Status status = db_->Put(write_options_fast_, cf_interventions_, key, payload);
    return status.ok();
}

bool FlightRecorder::log_telemetry_frame(
    uint64_t seq,
    uint64_t ts_ns,
    std::span<const float> positions,
    std::span<const float> torques
) noexcept {
    if (!db_ || !cf_telemetry_) return false;

    std::ostringstream key_stream;
    key_stream << "TELEM-" << std::setw(20) << std::setfill('0') << seq;
    std::string key = key_stream.str();

    std::string payload;
    payload.reserve(sizeof(uint64_t) * 2 + (positions.size() + torques.size()) * sizeof(float));
    
    payload.append(reinterpret_cast<const char*>(&seq), sizeof(seq));
    payload.append(reinterpret_cast<const char*>(&ts_ns), sizeof(ts_ns));
    payload.append(reinterpret_cast<const char*>(positions.data()), positions.size_bytes());
    payload.append(reinterpret_cast<const char*>(torques.data()), torques.size_bytes());

    rocksdb::Status status = db_->Put(write_options_fast_, cf_telemetry_, key, payload);
    return status.ok();
}

void FlightRecorder::flush() noexcept {
    if (db_ && cf_interventions_) {
        rocksdb::FlushOptions fo;
        db_->Flush(fo, cf_interventions_);
    }
}

} // namespace neurokinetic::storage
