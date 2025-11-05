/*
% ===========================================================================
% Filename: EstimatorPortN_MatlabSO.cpp
% Description:
%
% A middle transfer program to help MATLAB (.m) use the Linux .so library.
% It wraps the C API defined in EstimatorPortN.h into simple MEX entrypoints,
% so MATLAB can call initialize/estimate/terminate easily.
%
% Build (from MATLAB on Linux):
%   % puts MEX into: Estimator/EstimatorPortN_MatlabSO.mexa64
%   % links against: Estimator/EstimatorPortN.so
%   mex -v -IEstimator Estimator/EstimatorPortN_MatlabSO.cpp \
%       -LEstimator -lEstimatorPortN ...
%   (Your M-file demo already automates this and sets rpath to $ORIGIN.)
%
% The generated .mexa64 should stay in the same folder as EstimatorPortN.so.
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

#include "/home/unitree/software/matlab/extern/include/mex.h"
#include <cstring>

// --- Ensure Windows-specific EXPORT doesn't break Linux build ---
#ifdef EXPORT
#  undef EXPORT
#endif
#define EXPORT /* empty on Linux */

// Include the estimator port API.
// If the header uses extern "C" guards internally, great;
// otherwise we wrap here to avoid C++ name mangling.
extern "C" {
#include "EstimatorPortN.h"
}

// A single global estimator instance (mirrors your C/C++ demos)
static EstimatorPortN g_StateSpaceModel1;

// Helper: check input count/type
static void requireStringCmd(int nrhs, const mxArray *prhs[]) {
    if (nrhs < 1 || !mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("EstimatorPortN:badInput",
                          "First input must be a command string: 'initialize' | 'estimate' | 'terminate'.");
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    requireStringCmd(nrhs, prhs);

    // Fetch command
    char cmd[64] = {0};
    mxGetString(prhs[0], cmd, sizeof(cmd)-1);

    if (std::strcmp(cmd, "initialize") == 0) {
        StateSpaceModel1_Initialization(&g_StateSpaceModel1);
        if (nlhs > 0) {
            plhs[0] = mxCreateLogicalScalar(true);
        }
        return;
    }

    if (std::strcmp(cmd, "estimate") == 0) {
        if (nrhs != 2) {
            mexErrMsgIdAndTxt("EstimatorPortN:badInput",
                              "'estimate' requires one argument: observation vector (Nz x 1 or 1 x Nz).");
        }

        const mxArray *obsArr = prhs[1];
        if (!mxIsDouble(obsArr) || mxIsComplex(obsArr)) {
            mexErrMsgIdAndTxt("EstimatorPortN:type",
                              "Observation must be real double.");
        }

        // Expect length == Nz
        mwSize nEl = mxGetNumberOfElements(obsArr);
        if (static_cast<int>(nEl) != g_StateSpaceModel1.Nz) {
            mexErrMsgIdAndTxt("EstimatorPortN:dim",
                              "Observation length (%d) does not match Nz (%d).",
                              static_cast<int>(nEl), g_StateSpaceModel1.Nz);
        }

        // Prepare output xhat (Nx x 1)
        plhs[0] = mxCreateDoubleMatrix(g_StateSpaceModel1.Nx, 1, mxREAL);

        double *obs = mxGetPr(obsArr);
        double *xhat = mxGetPr(plhs[0]);

        StateSpaceModel1_EstimatorPort(obs, xhat, &g_StateSpaceModel1);
        return;
    }

    if (std::strcmp(cmd, "terminate") == 0) {
        StateSpaceModel1_EstimatorPortTermination(&g_StateSpaceModel1);
        if (nlhs > 0) {
            plhs[0] = mxCreateLogicalScalar(true);
        }
        return;
    }

    mexErrMsgIdAndTxt("EstimatorPortN:unknownCmd",
                      "Unknown command '%s'. Use 'initialize' | 'estimate' | 'terminate'.", cmd);
}
