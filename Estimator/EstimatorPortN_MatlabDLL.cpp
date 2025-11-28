/*
% ===========================================================================
% Filename: EstimatorPortN_MatlabDLL.cpp
% Description:
%
% A middle tranfer program to help matlab .m to use DLL.
% Because the estimator port file EstimatorPortN.h use struct and funcional element, 
% the matlab is not clever enough to directly use the generated EstimatorPortN.dll.
% 
% This file call EstimatorPortN.dll and package it into simpler method.
%
% Matlab could compile this file with command:
% mex -I'D:\Temp\Project\EstimatorFrame\Estimator' Estimator\EstimatorPortN_MatlabDLL.cpp 'D:\Temp\Project\EstimatorFrame\Estimator\EstimatorPortN.lib' -outdir 'D:\Temp\Project\EstimatorFrame\Estimator'
% and generate EstimatorPortN_MatlabDLL.mexw64.
%
% EstimatorPortN_MatlabDLL.mexw64 need to be in the same path as EstimatorPortN.dll.
% 
% Then, after add .mexw64's path into matlab, functions in this file can be used.
% 
% The matlab demo could refer to M_DLL_Demo.m.
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
#include "C:\Program Files\MATLAB\R2025a\extern\include\mex.h"
#include <windows.h>

// Redefine EXPORT macro to __declspec(dllimport)
#ifdef EXPORT
#undef EXPORT
#endif
#define EXPORT __declspec(dllimport)

#include "EstimatorPortN.h"

// Global variables
EstimatorPortN StateSpaceModel_Demo_;

// MEX function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    char *command;

    // Check for proper number of arguments
    if (nrhs < 1 || !mxIsChar(prhs[0]))
        mexErrMsgTxt("First input should be a command string.");

    // Get the command
    command = mxArrayToString(prhs[0]);

    if (strcmp(command, "initialize") == 0) {
        // Initialize the estimator
        StateSpaceModel_Demo_Initialization(&StateSpaceModel_Demo_);

        // Return success
        plhs[0] = mxCreateLogicalScalar(true);

    } 
    else if (strcmp(command, "estimate") == 0) {
        // Check for proper number of arguments
        if (nrhs != 3)
            mexErrMsgTxt("'estimate' requires: observation (vector) and timestamp (double).");

        // Get the observation data
        double *observation = mxGetPr(prhs[1]);
        size_t obsLength = mxGetNumberOfElements(prhs[1]);

        if (obsLength != (size_t)StateSpaceModel_Demo_.Nz) {
            mexErrMsgTxt("Observation data size does not match Nz.");
        }

        // Get timestamp
        if (!mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1) {
            mexErrMsgTxt("Timestamp must be a scalar double.");
        }
        double tstamp = *mxGetPr(prhs[2]);

        // Prepare output
        plhs[0] = mxCreateDoubleMatrix(StateSpaceModel_Demo_.Nx, 1, mxREAL);
        double *estimatedState = mxGetPr(plhs[0]);

        // Call the EstimatorPort function
        StateSpaceModel_Demo_EstimatorPort(observation, tstamp, &StateSpaceModel_Demo_);

        for (int k = 0; k < StateSpaceModel_Demo_.Nx; ++k) {
            estimatedState[k] = StateSpaceModel_Demo_.EstimatedState[k];
        }

    } else if (strcmp(command, "terminate") == 0) {
        // Call the EstimatorPortTermination function
        StateSpaceModel_Demo_EstimatorPortTermination(&StateSpaceModel_Demo_);

        // Return success
        plhs[0] = mxCreateLogicalScalar(true);

    } else {
        mexErrMsgTxt("Unknown command.");
    }

    // Free the command string
    mxFree(command);
}
