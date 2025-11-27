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
typedef void (*EP_EstimatorPort_Type)(double *, double *, struct EstimatorPortN *);
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
    double EstimatedState[DATA_ROWS][State_Dimension];

    readDataToObservation(ReadFileData);

    // Load DLL
    HMODULE hEstimatorDll = LoadLibrary(DLL_PATH);
    if (hEstimatorDll == NULL) {
        std::cerr << "Unable to load EstimatorPortN.dll" << std::endl;
        return EXIT_FAILURE;
    }
    // Obtain DLL function pointers
    EP_Initialization_Type StateSpaceModel1_Initialization = (EP_Initialization_Type)GetProcAddress(hEstimatorDll, "StateSpaceModel1_Initialization");
    EP_EstimatorPort_Type StateSpaceModel1_EstimatorPort = (EP_EstimatorPort_Type)GetProcAddress(hEstimatorDll, "StateSpaceModel1_EstimatorPort");
    EP_Termination_Type StateSpaceModel1_EstimatorPortTermination = (EP_Termination_Type)GetProcAddress(hEstimatorDll, "StateSpaceModel1_EstimatorPortTermination");
    if (!StateSpaceModel1_Initialization || !StateSpaceModel1_EstimatorPort || !StateSpaceModel1_EstimatorPortTermination) {
        std::cerr << "Unable to find required functions in DLL" << std::endl;
        FreeLibrary(hEstimatorDll);
        return EXIT_FAILURE;
    }
    // Initiate EstimatorPortN struct
    EstimatorPortN StateSpaceModel1_;

    // StateSpaceModel1_ based estimation
    StateSpaceModel1_Initialization(&StateSpaceModel1_);
    for (int i = 0; i < DATA_ROWS; ++i) {
        for (int j = 0; j < Observation_Dimension; ++j) {
            EstimatorObservation[i][j] = ReadFileData[i][j + 1];
        }
        StateSpaceModel1_EstimatorPort(EstimatorObservation[i], EstimatedState[i], &StateSpaceModel1_);
        WriteFileData[i][0] = ReadFileData[i][0];
        for (int j = 0; j < State_Dimension; ++j) {
            WriteFileData[i][j + 1] = EstimatedState[i][j];
        }
    }
    StateSpaceModel1_EstimatorPortTermination(&StateSpaceModel1_);
    // Free DLL
    FreeLibrary(hEstimatorDll);
    std::cout << "StateSpaceModel1_ finished..." << std::endl;
    // StateSpaceModel1_ based estimation

    WriteDataToFile(WriteFileData);

    std::cout << "Program terminated..." << std::endl;

    return 0;
}
