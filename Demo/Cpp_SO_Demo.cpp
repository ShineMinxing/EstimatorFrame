/*
% ===========================================================================
% Filename: Cpp_SO_Demo.cpp
% Description:
%   Linux demo (C++) for calling EstimatorPortN via runtime-loaded .so
%   Loads Estimator/EstimatorPortN.so with dlopen(), resolves symbols
%   with dlsym(), then runs the same estimation flow as Windows version.
%
% Initial version: Minxing Sun
% Unit: UCAS, Institute of Optics And Electronics, Lab 1, Class of 2020
% Email: 401435318@qq.com
% Date: November 5, 2025
% 
% Updates:
% Unit:
% Email:
% Date:
% ===========================================================================
*/

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <filesystem>   // C++17
#include <dlfcn.h>      // dlopen, dlsym, dlclose

#ifdef __cplusplus
extern "C" {
#endif
#include "EstimatorPortN.h"  // reachable via include path: ${PROJECT}/Estimator
#ifdef __cplusplus
}
#endif

// ---- Macro definitions (Linux 路径用正斜杠) ----
#define INPUT_PATH  "ObservationData/DoubleReflectorTrace/Trace1000.txt"
#define OUTPUT_DIR  "EstimationResult"
#define SO_PATH     "Output/EstimatorPortN.so"

#define DATA_ROWS             1000
#define READ_DATA_COLUMNS     3
#define WRITE_DATA_COLUMNS    5
#define Observation_Dimension 2
#define State_Dimension       4

// ---- Function pointer types that match the .so exports ----
using EP_Initialization_Type = void (*)(struct EstimatorPortN *);
using EP_EstimatorPort_Type  = void (*)(double *, double, struct EstimatorPortN *);
using EP_Termination_Type    = void (*)(struct EstimatorPortN *);

// ---- IO helpers ----
static void readDataToObservation(double ReadFileData[DATA_ROWS][READ_DATA_COLUMNS]) {
    std::ifstream fin(INPUT_PATH);
    if (!fin) {
        std::cerr << "Unable to open file: " << INPUT_PATH << std::endl;
        std::exit(EXIT_FAILURE);
    }
    for (int i = 0; i < DATA_ROWS; ++i) {
        for (int j = 0; j < READ_DATA_COLUMNS; ++j) {
            if (!(fin >> ReadFileData[i][j])) {
                std::cerr << "Error reading file at row " << i << " column " << j << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }
}

static void writeEstimationToFile(const double WriteFileData[DATA_ROWS][WRITE_DATA_COLUMNS]) {
    // Ensure output dir exists
    std::error_code ec;
    std::filesystem::create_directories(OUTPUT_DIR, ec);
    if (ec) {
        std::cerr << "Failed to create output dir: " << OUTPUT_DIR
                  << " (" << ec.message() << ")\n";
        std::exit(EXIT_FAILURE);
    }

    // Timestamp
    std::time_t t = std::time(nullptr);
    std::tm tm_now{};
    localtime_r(&t, &tm_now);

    char ts[20];
    std::strftime(ts, sizeof(ts), "%Y%m%d%H%M%S", &tm_now);

    // Output path
    std::filesystem::path outPath = std::filesystem::path(OUTPUT_DIR) /
                                    ("EstimationResult_" + std::string(ts) + ".txt");

    std::ofstream fout(outPath);
    if (!fout) {
        std::cerr << "Unable to create file: " << outPath.string() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    fout.setf(std::ios::fixed);
    fout << std::setprecision(6);
    for (int i = 0; i < DATA_ROWS; ++i) {
        for (int j = 0; j < WRITE_DATA_COLUMNS; ++j) {
            fout << WriteFileData[i][j] << (j + 1 < WRITE_DATA_COLUMNS ? ' ' : '\n');
        }
    }
    std::cout << "Estimation data has been written to " << outPath.string() << std::endl;
}

// ---- main ----
int main() {
    // 1) Read input
    double ReadFileData[DATA_ROWS][READ_DATA_COLUMNS];
    double WriteFileData[DATA_ROWS][WRITE_DATA_COLUMNS];
    double EstimatorObservation[DATA_ROWS][Observation_Dimension];

    readDataToObservation(ReadFileData);

    // 2) Load .so with dlopen (use relative path so不依赖rpath)
    dlerror(); // clear
    void* handle = dlopen(SO_PATH, RTLD_NOW);
    if (!handle) {
        std::cerr << "dlopen failed: " << dlerror() << "\n"
                  << "Tried: " << SO_PATH
                  << " (run from project root so the relative path is valid)\n";
        return EXIT_FAILURE;
    }

    // 3) Resolve symbols
    auto StateSpaceModel_Demo_Initialization =
        reinterpret_cast<EP_Initialization_Type>(dlsym(handle, "StateSpaceModel_Demo_Initialization"));
    auto StateSpaceModel_Demo_EstimatorPort =
        reinterpret_cast<EP_EstimatorPort_Type>(dlsym(handle, "StateSpaceModel_Demo_EstimatorPort"));
    auto StateSpaceModel_Demo_EstimatorPortTermination =
        reinterpret_cast<EP_Termination_Type>(dlsym(handle, "StateSpaceModel_Demo_EstimatorPortTermination"));

    const char* err = dlerror();
    if (err || !StateSpaceModel_Demo_Initialization || !StateSpaceModel_Demo_EstimatorPort || !StateSpaceModel_Demo_EstimatorPortTermination) {
        std::cerr << "dlsym error: " << (err ? err : "one or more symbols are null") << std::endl;
        dlclose(handle);
        return EXIT_FAILURE;
    }

    // 4) Estimation loop
    EstimatorPortN StateSpaceModel_Demo_;
    StateSpaceModel_Demo_Initialization(&StateSpaceModel_Demo_);

    for (int i = 0; i < DATA_ROWS; ++i) {
        // ReadFileData cols 1..2 -> obs[0..1]
        for (int j = 0; j < Observation_Dimension; ++j) {
            EstimatorObservation[i][j] = ReadFileData[i][j + 1];
        }
        // run estimator
        StateSpaceModel_Demo_EstimatorPort(EstimatorObservation[i], ReadFileData[i][0], &StateSpaceModel_Demo_);
        // write record: time + estimated state
        WriteFileData[i][0] = ReadFileData[i][0];
        for (int j = 0; j < State_Dimension; ++j) {
            WriteFileData[i][j + 1] = StateSpaceModel_Demo_.EstimatedState[j];
        }
    }

    StateSpaceModel_Demo_EstimatorPortTermination(&StateSpaceModel_Demo_);
    dlclose(handle);
    std::cout << "StateSpaceModel_Demo_ finished..." << std::endl;

    // 5) Dump output
    writeEstimationToFile(WriteFileData);
    std::cout << "Program terminated..." << std::endl;
    return 0;
}
