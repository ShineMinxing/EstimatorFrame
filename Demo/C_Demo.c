/*
% ===========================================================================
% Filename: C_Demo.c
% Description:
%
% A demo for demonstrating how to call estimator port in .c form.
%
% All source files in \Estimator and its subdirectory are used. 
% Make sure there is at least one estimator method in the subdirectory, like 
% Estimator1001_Kalman.c and Estimator1001_Kalman.h.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../Estimator/EstimatorPortN.h"

// Macro configuration
#define INPUT_PATH "ObservationData/DoubleReflectorTrace/Trace1000.txt"
#define OUTPUT_PATH "EstimationResult"
#define DATA_ROWS 1000
#define READ_DATA_COLUMNS 3
#define WRITE_DATA_COLUMNS 5
#define Observation_Dimension 2
#define State_Dimension 4

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
    double EstimatedState[DATA_ROWS][State_Dimension];

    readDataToObservation(ReadFileData);

    // StateSpaceModel1_ based estimation
    StateSpaceModel1_Initialization(&StateSpaceModel1_);
    for (int i = 0; i < DATA_ROWS; i++) {
        for (int j = 0; j < Observation_Dimension; j++) {
            EstimatorObservation[i][j] = ReadFileData[i][j+1];
        }
        StateSpaceModel1_EstimatorPort(EstimatorObservation[i], EstimatedState[i], &StateSpaceModel1_);
        WriteFileData[i][0] = ReadFileData[i][0];
        for (int j = 0; j < State_Dimension; j++) {
            WriteFileData[i][j+1] = EstimatedState[i][j];
        }
    }
    StateSpaceModel1_EstimatorPortTermination(&StateSpaceModel1_);
    printf("StateSpaceModel1_ finished...\n");
    // StateSpaceModel1_ based estimation
    
    WriteDataToFile(WriteFileData);

    printf("Program terminated...\n");

    return 0;
}