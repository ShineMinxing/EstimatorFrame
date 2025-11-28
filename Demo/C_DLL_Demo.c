/*
% ===========================================================================
% Filename: C_DLL_Demo.c
% Description:
% 
% A demo for demonstrating how to call estimator port in dll form.
%
% Estimator\EstimatorPortN.dll and Estimator\EstimatorPortN.lib are used.
%
% Mainly use .vscode\tasks.json and .vscode\build_c_dll_demo.bat to compile
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../Estimator/EstimatorPortN.h"

// Macro configuration
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

    FILE *file = fopen(INPUT_PATH, "r");
    if (file == NULL) {
        perror("Unable to open file");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < DATA_ROWS; i++) {
        for (int j = 0; j < READ_DATA_COLUMNS; j++) {
            if (fscanf(file, "%lf", &ReadFileData[i][j]) != 1) {
                fprintf(stderr, "Error reading file at row %d column %d\n", i, j);
                fclose(file);
                exit(EXIT_FAILURE);
            }
        }
    }
    fclose(file);
}

void WriteDataToFile(double WriteFileData[DATA_ROWS][WRITE_DATA_COLUMNS]) {
    
    char fullOutputFilePath[260];
    time_t rawtime;
    struct tm *timeinfo;
    char timeBuffer[20];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timeBuffer, sizeof(timeBuffer), "%Y%m%d%H%M%S", timeinfo);
    
    snprintf(fullOutputFilePath, sizeof(fullOutputFilePath), "%s/EstimationResult_%s.txt", OUTPUT_PATH, timeBuffer);

    FILE *file = fopen(fullOutputFilePath, "w");
    if (file == NULL) {
        perror("Unable to create file");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < DATA_ROWS; i++) {
        for (int j = 0; j < WRITE_DATA_COLUMNS; j++) {
            fprintf(file, "%.6lf ", WriteFileData[i][j]);
        }
        fprintf(file, "\n");
    }

    fclose(file);

    printf("Estimation data has been written to %s\n",fullOutputFilePath);
}

int main() {
    double ReadFileData[DATA_ROWS][READ_DATA_COLUMNS];
    double WriteFileData[DATA_ROWS][WRITE_DATA_COLUMNS];
    double EstimatorObservation[DATA_ROWS][Observation_Dimension];
    
    readDataToObservation(ReadFileData);

    // Load DLL
    HMODULE hEstimatorDll = LoadLibrary(DLL_PATH);
    if (hEstimatorDll == NULL) {
        perror("Unable to load EstimatorPortN.dll");
        return EXIT_FAILURE;
    }
    // Obtain DLL ffunction pointer
    EP_Initialization_Type StateSpaceModel_Demo_Initialization = (EP_Initialization_Type)GetProcAddress(hEstimatorDll, "StateSpaceModel_Demo_Initialization");
    EP_EstimatorPort_Type StateSpaceModel_Demo_EstimatorPort = (EP_EstimatorPort_Type)GetProcAddress(hEstimatorDll, "StateSpaceModel_Demo_EstimatorPort");
    EP_Termination_Type StateSpaceModel_Demo_EstimatorPortTermination = (EP_Termination_Type)GetProcAddress(hEstimatorDll, "StateSpaceModel_Demo_EstimatorPortTermination");
    if (!StateSpaceModel_Demo_Initialization || !StateSpaceModel_Demo_EstimatorPort || !StateSpaceModel_Demo_EstimatorPortTermination) {
        perror("Unable to find required functions in DLL");
        FreeLibrary(hEstimatorDll);
        return EXIT_FAILURE;
    }
    // Initiate EstimatorPortN struct
    EstimatorPortN StateSpaceModel_Demo_;

    // StateSpaceModel_Demo_ based estimation
    StateSpaceModel_Demo_Initialization(&StateSpaceModel_Demo_);
    for (int i = 0; i < DATA_ROWS; i++) {
        for (int j = 0; j < Observation_Dimension; j++) {
            EstimatorObservation[i][j] = ReadFileData[i][j+1];
        }
        StateSpaceModel_Demo_EstimatorPort(EstimatorObservation[i], ReadFileData[i][0], &StateSpaceModel_Demo_);
        WriteFileData[i][0] = ReadFileData[i][0];
        for (int j = 0; j < State_Dimension; j++) {
            WriteFileData[i][j+1] = StateSpaceModel_Demo_.EstimatedState[j];
        }
    }
    StateSpaceModel_Demo_EstimatorPortTermination(&StateSpaceModel_Demo_);
    FreeLibrary(hEstimatorDll);
    printf("StateSpaceModel_Demo_ finished...\n");
    // StateSpaceModel_Demo_ based estimation

    
    WriteDataToFile(WriteFileData);

    printf("Program terminated...\n");

    return 0;
}