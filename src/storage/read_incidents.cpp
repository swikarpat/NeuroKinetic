#include "storage/flight_recorder.hpp"
#include <rocksdb/db.h>
#include <rocksdb/options.h>
#include <iostream>
#include <vector>
#include <memory>
#include <cstring>

using namespace neurokinetic::storage;

int main() {
    rocksdb::DBOptions db_options;
    std::string db_path = "/tmp/neurokinetic_flight_recorder";

    std::vector<rocksdb::ColumnFamilyDescriptor> column_families;
    column_families.emplace_back(rocksdb::kDefaultColumnFamilyName, rocksdb::ColumnFamilyOptions());
    column_families.emplace_back("cf_cbf_interventions", rocksdb::ColumnFamilyOptions());
    column_families.emplace_back("cf_flight_telemetry", rocksdb::ColumnFamilyOptions());

    std::vector<rocksdb::ColumnFamilyHandle*> handles;
    std::unique_ptr<rocksdb::DB> db;

    rocksdb::Status status = rocksdb::DB::OpenForReadOnly(
        db_options, db_path, column_families, &handles, &db
    );

    if (!status.ok() || !db) {
        std::cout << "[]\n";
        return 0;
    }

    std::unique_ptr<rocksdb::Iterator> it(db->NewIterator(rocksdb::ReadOptions(), handles[1]));
    
    std::cout << "[\n";
    bool first = true;
    size_t count = 0;

    for (it->SeekToLast(); it->Valid() && count < 50; it->Prev()) {
        if (it->value().size() == sizeof(InterventionRecord)) {
            InterventionRecord rec;
            std::memcpy(&rec, it->value().data(), sizeof(InterventionRecord));

            if (!first) std::cout << ",\n";
            first = false;

            std::cout << "  {\n"
                      << "    \"key\": \"" << it->key().ToString() << "\",\n"
                      << "    \"sequence\": " << rec.sequence_number << ",\n"
                      << "    \"timestamp_ns\": " << rec.timestamp_ns << ",\n"
                      << "    \"barrier_margin\": " << rec.barrier_margin << ",\n"
                      << "    \"commanded_torque\": " << rec.commanded_joint5_torque << ",\n"
                      << "    \"safe_torque\": " << rec.safe_joint5_torque << ",\n"
                      << "    \"active_mask\": " << rec.active_mask << "\n"
                      << "  }";
            count++;
        }
    }
    std::cout << "\n]\n";

    for (auto* h : handles) delete h;
    return 0;
}
