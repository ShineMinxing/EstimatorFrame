/*
% ===========================================================================
% Filename: Cpp_DLL_Demo.cpp
% Description:
%
% A demo for demonstrating how to call estimator port in dll form.
%
% Estimator\EstimatorPortN.dll and Estimator\EstimatorPortN.lib are used.
%
% Mainly use .vscode\tasks.json and .vscode\build_cpp_dll_demo.bat to compile
%
% Initial version: Minxing Sun
% Unit: UCAS, Institute of Optics And Electronics, Lab 1, Class of 2020
% Email: 401435318@qq.com
% Date: November 16, 2024
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

#ifdef __cplusplus
extern "C" {
#endif

#include "EstimatorPortN.h"

#ifdef __cplusplus
}
#endif

// Macro definitions
#define INPUT_PATH "ObservationData/DoubleReflectorTrace/Trace1000.txt"
#define OUTPUT_PATH "EstimationResult"
#define DLL_PATH "Output/EstimatorPortN.dll"
#define DATA_ROWS 1000
#define READ_DATA_COLUMNS 3
#define WRITE_DATA_COLUMNS 5
#define Observation_Dimension 2
#define State_Dimension 4

// For Using dll file
#include <windows.h>
typedef void (*EP_Initialization_Type)(struct EstimatorPortN *);
typedef void (*EP_EstimatorPort_Type)(double *, double, struct EstimatorPortN *);
typedef void (*EP_Termination_Type)(struct EstimatorPortN *);

void readDataToObservation(double ReadFileData[DATA_ROWS][READ_DATA_COLUMNS]) {

    std::ifstream file(INPUT_PATH);
    if (!file) {
        std::cerr << "Unable to open file: " << INPUT_PATH << std::endl;
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < DATA_ROWS; ++i) {
        for (int j = 0; j < READ_DATA_COLUMNS; ++j) {
            if (!(file >> ReadFileData[i][j])) {
                std::cerr << "Error reading file at row " << i << " column " << j << std::endl;
                file.close();
                exit(EXIT_FAILURE);
            }
        }
    }
    file.close();
}

void WriteDataToFile(double WriteFileData[DATA_ROWS][WRITE_DATA_COLUMNS]) {

    std::ostringstream timeBuffer;
    std::time_t rawtime;
    std::tm *timeinfo;
    char timeStr[20];

    std::time(&rawtime);
    timeinfo = std::localtime(&rawtime);
    std::strftime(timeStr, sizeof(timeStr), "%Y%m%d%H%M%S", timeinfo);

    std::string fullOutputFilePath = std::string(OUTPUT_PATH) + "/EstimationResult_" + timeStr + ".txt";

    std::ofstream file(fullOutputFilePath);
    if (!file) {
        std::cerr << "Unable to create file: " << fullOutputFilePath << std::endl;
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < DATA_ROWS; ++i) {
        for (int j = 0; j < WRITE_DATA_COLUMNS; ++j) {
            file << std::fixed << std::setprecision(6) << WriteFileData[i][j] << " ";
        }
        file << "\n";
    }

    file.close();
    std::cout << "Estimation data has been written to " << fullOutputFilePath << std::endl;
}

int main() {
    double ReadFileData[DATA_ROWS][READ_DATA_COLUMNS];
    double WriteFileData[DATA_ROWS][WRITE_DATA_COLUMNS];
    double EstimatorObservation[DATA_ROWS][Observation_Dimension];

    readDataToObservation(ReadFileData);

    // Load DLL
    HMODULE hEstimatorDll = LoadLibrary(DLL_PATH);
    if (hEstimatorDll == NULL) {
        std::cerr << "Unable to load EstimatorPortN.dll" << std::endl;
        return EXIT_FAILURE;
    }
    // Obtain DLL function pointers
    EP_Initialization_Type StateSpaceModel_Demo_Initialization = (EP_Initialization_Type)GetProcAddress(hEstimatorDll, "StateSpaceModel_Demo_Initialization");
    EP_EstimatorPort_Type StateSpaceModel_Demo_EstimatorPort = (EP_EstimatorPort_Type)GetProcAddress(hEstimatorDll, "StateSpaceModel_Demo_EstimatorPort");
    EP_Termination_Type StateSpaceModel_Demo_EstimatorPortTermination = (EP_Termination_Type)GetProcAddress(hEstimatorDll, "StateSpaceModel_Demo_EstimatorPortTermination");
    if (!StateSpaceModel_Demo_Initialization || !StateSpaceModel_Demo_EstimatorPort || !StateSpaceModel_Demo_EstimatorPortTermination) {
        std::cerr << "Unable to find required functions in DLL" << std::endl;
        FreeLibrary(hEstimatorDll);
        return EXIT_FAILURE;
    }
    // Initiate EstimatorPortN struct
    EstimatorPortN StateSpaceModel_Demo_;

    // StateSpaceModel_Demo_ based estimation
    StateSpaceModel_Demo_Initialization(&StateSpaceModel_Demo_);
    for (int i = 0; i < DATA_ROWS; ++i) {
        for (int j = 0; j < Observation_Dimension; ++j) {
            EstimatorObservation[i][j] = ReadFileData[i][j + 1];
        }
        StateSpaceModel_Demo_EstimatorPort(EstimatorObservation[i], ReadFileData[i][0], &StateSpaceModel_Demo_);
        WriteFileData[i][0] = ReadFileData[i][0];
        for (int j = 0; j < State_Dimension; ++j) {
            WriteFileData[i][j + 1] = StateSpaceModel_Demo_.EstimatedState[j];
        }
    }
    StateSpaceModel_Demo_EstimatorPortTermination(&StateSpaceModel_Demo_);
    // Free DLL
    FreeLibrary(hEstimatorDll);
    std::cout << "StateSpaceModel_Demo_ finished..." << std::endl;
    // StateSpaceModel_Demo_ based estimation

    WriteDataToFile(WriteFileData);

    std::cout << "Program terminated..." << std::endl;

    return 0;
}
