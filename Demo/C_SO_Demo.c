/*
% ===========================================================================
% Filename: C_SO_Demo.c
% Description:
%
% A Linux demo showing how to call EstimatorPortN via runtime loading (.so).
%
% It loads Estimator/EstimatorPortN.so at runtime with dlopen(),
% resolves functions by dlsym(), and runs exactly the same estimation flow
% as your Windows C_DLL_Demo.c version.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <dlfcn.h>          // dlopen, dlsym, dlclose
#include <sys/stat.h>       // stat, mkdir
#include <sys/types.h>
#include <errno.h>
#include "../Estimator/EstimatorPortN.h"   // struct EstimatorPortN

/* ---------------- Macro configuration (Linux paths use '/') ---------------- */
#define INPUT_PATH  "ObservationData/DoubleReflectorTrace/Trace1000.txt"
#define OUTPUT_DIR  "EstimationResult"
#define SO_PATH     "Output/EstimatorPortN.so"

#define DATA_ROWS            1000
#define READ_DATA_COLUMNS    3
#define WRITE_DATA_COLUMNS   5
#define Observation_Dimension 2
#define State_Dimension       4

/* ---------------- Types of the functions exported by the .so ----------------
   Signatures must match the implementation inside EstimatorPortN */
typedef void (*EP_Initialization_Type)(struct EstimatorPortN *);
typedef void (*EP_EstimatorPort_Type)(double *, double, struct EstimatorPortN *);
typedef void (*EP_Termination_Type)(struct EstimatorPortN *);

/* ----------------------------- Small utilities ----------------------------- */
static int ensure_dir_exists(const char* path)
{
    struct stat st;
    if (stat(path, &st) == 0) {
        if ((st.st_mode & S_IFMT) == S_IFDIR) return 0;
        fprintf(stderr, "Path exists but is not a directory: %s\n", path);
        return -1;
    }
    if (mkdir(path, 0755) == 0) return 0;
    if (errno == EEXIST) return 0;
    perror("mkdir");
    return -1;
}

static void readDataToObservation(double ReadFileData[DATA_ROWS][READ_DATA_COLUMNS])
{
    FILE *fp = fopen(INPUT_PATH, "r");
    if (!fp) {
        perror("Unable to open input file");
        fprintf(stderr, "Tried: %s\n", INPUT_PATH);
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < DATA_ROWS; ++i) {
        for (int j = 0; j < READ_DATA_COLUMNS; ++j) {
            if (fscanf(fp, "%lf", &ReadFileData[i][j]) != 1) {
                fprintf(stderr, "Error reading file at row %d col %d\n", i, j);
                fclose(fp);
                exit(EXIT_FAILURE);
            }
        }
    }
    fclose(fp);
}

static void writeEstimationToFile(const double WriteFileData[DATA_ROWS][WRITE_DATA_COLUMNS])
{
    if (ensure_dir_exists(OUTPUT_DIR) != 0) {
        fprintf(stderr, "Failed to create output dir: %s\n", OUTPUT_DIR);
        exit(EXIT_FAILURE);
    }

    time_t t = time(NULL);
    struct tm tm_now;
    localtime_r(&t, &tm_now);

    char ts[20];
    strftime(ts, sizeof(ts), "%Y%m%d%H%M%S", &tm_now);

    char outpath[512];
    snprintf(outpath, sizeof(outpath), "%s/EstimationResult_%s.txt", OUTPUT_DIR, ts);

    FILE *fp = fopen(outpath, "w");
    if (!fp) {
        perror("Unable to create output file");
        fprintf(stderr, "Tried: %s\n", outpath);
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < DATA_ROWS; ++i) {
        for (int j = 0; j < WRITE_DATA_COLUMNS; ++j) {
            fprintf(fp, "%.6f%c", WriteFileData[i][j], (j+1<WRITE_DATA_COLUMNS)?' ':'\n');
        }
    }
    fclose(fp);
    printf("Estimation data has been written to %s\n", outpath);
}

/* ----------------------------------- main ---------------------------------- */
int main(void)
{
    /* 1) Load input data */
    double ReadFileData[DATA_ROWS][READ_DATA_COLUMNS];
    double WriteFileData[DATA_ROWS][WRITE_DATA_COLUMNS];
    double EstimatorObservation[DATA_ROWS][Observation_Dimension];

    readDataToObservation(ReadFileData);

    /* 2) dlopen the shared library */
    dlerror(); // clear any existing error
    void* handle = dlopen(SO_PATH, RTLD_NOW);
    if (!handle) {
        fprintf(stderr, "dlopen failed: %s\n", dlerror());
        fprintf(stderr, "Tried: %s (run from project root so relative path is valid)\n", SO_PATH);
        return EXIT_FAILURE;
    }

    /* 3) Resolve required symbols */
    EP_Initialization_Type StateSpaceModel_Demo_Initialization =
        (EP_Initialization_Type)dlsym(handle, "StateSpaceModel_Demo_Initialization");
    EP_EstimatorPort_Type StateSpaceModel_Demo_EstimatorPort =
        (EP_EstimatorPort_Type)dlsym(handle, "StateSpaceModel_Demo_EstimatorPort");
    EP_Termination_Type StateSpaceModel_Demo_EstimatorPortTermination =
        (EP_Termination_Type)dlsym(handle, "StateSpaceModel_Demo_EstimatorPortTermination");

    const char* err = dlerror();
    if (err) {
        fprintf(stderr, "dlsym error: %s\n", err);
        dlclose(handle);
        return EXIT_FAILURE;
    }
    if (!StateSpaceModel_Demo_Initialization || !StateSpaceModel_Demo_EstimatorPort || !StateSpaceModel_Demo_EstimatorPortTermination) {
        fprintf(stderr, "Required functions not found in %s\n", SO_PATH);
        dlclose(handle);
        return EXIT_FAILURE;
    }

    /* 4) Run estimation exactly like Windows version */
    struct EstimatorPortN StateSpaceModel_Demo_;
    StateSpaceModel_Demo_Initialization(&StateSpaceModel_Demo_);

    for (int i = 0; i < DATA_ROWS; ++i) {
        /* columns 1..2 -> observation[0..1] */
        for (int j = 0; j < Observation_Dimension; ++j) {
            EstimatorObservation[i][j] = ReadFileData[i][j + 1];
        }

        /* call estimator; state goes into EstimatedState[i] */
        StateSpaceModel_Demo_EstimatorPort(EstimatorObservation[i], ReadFileData[i][0], &StateSpaceModel_Demo_);

        /* time in col0; estimated state in cols 1..4 */
        WriteFileData[i][0] = ReadFileData[i][0];
        for (int j = 0; j < State_Dimension; ++j) {
            WriteFileData[i][j + 1] = StateSpaceModel_Demo_.EstimatedState[j];
        }
    }

    StateSpaceModel_Demo_EstimatorPortTermination(&StateSpaceModel_Demo_);
    dlclose(handle);
    printf("StateSpaceModel_Demo_ finished...\n");

    /* 5) Dump output */
    writeEstimationToFile(WriteFileData);
    printf("Program terminated...\n");
    return 0;
}
