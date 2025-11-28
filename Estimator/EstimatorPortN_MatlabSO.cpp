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
// ======= 仅替换整文件里这部分 =======
#include <cstring>

#ifdef EXPORT
#  undef EXPORT
#endif
#define EXPORT /* empty on Linux */

extern "C" {
#include "EstimatorPortN.h"
}

// 全局实例
static EstimatorPortN g_StateSpaceModel_Demo;

static void requireStringCmd(int nrhs, const mxArray *prhs[]) {
    if (nrhs < 1 || !mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("EstimatorPortN:badInput",
                          "First input must be a command string: 'initialize' | 'estimate' | 'terminate'.");
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    requireStringCmd(nrhs, prhs);

    char cmd[64] = {0};
    mxGetString(prhs[0], cmd, sizeof(cmd)-1);

    if (std::strcmp(cmd, "initialize") == 0) {
        StateSpaceModel_Demo_Initialization(&g_StateSpaceModel_Demo);
        if (nlhs > 0) plhs[0] = mxCreateLogicalScalar(true);
        return;
    }

    if (std::strcmp(cmd, "estimate") == 0) {
        // 现在需要两个实参：obs 向量 + 时间戳
        if (nrhs != 3) {
            mexErrMsgIdAndTxt("EstimatorPortN:badInput",
                              "'estimate' requires 2 args: observation (Nz) and timestamp (double).");
        }

        const mxArray *obsArr = prhs[1];
        if (!mxIsDouble(obsArr) || mxIsComplex(obsArr)) {
            mexErrMsgIdAndTxt("EstimatorPortN:type",
                              "Observation must be real double.");
        }

        mwSize nEl = mxGetNumberOfElements(obsArr);
        if (static_cast<int>(nEl) != g_StateSpaceModel_Demo.Nz) {
            mexErrMsgIdAndTxt("EstimatorPortN:dim",
                              "Observation length (%d) != Nz (%d).",
                              static_cast<int>(nEl), g_StateSpaceModel_Demo.Nz);
        }

        // 读取时间戳（标量 double）
        if (!mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1) {
            mexErrMsgIdAndTxt("EstimatorPortN:time",
                              "Timestamp must be a scalar double.");
        }
        double tstamp = *mxGetPr(prhs[2]);

        // 输出 xhat（Nx x 1）
        plhs[0] = mxCreateDoubleMatrix(g_StateSpaceModel_Demo.Nx, 1, mxREAL);
        double *xhat = mxGetPr(plhs[0]);

        // 调用新版 C 接口：第二参数是时间戳
        double *obs = mxGetPr(obsArr);
        StateSpaceModel_Demo_EstimatorPort(obs, tstamp, &g_StateSpaceModel_Demo);

        // 从结构体里拷贝 EstimatedState 到 xhat
        for (int k = 0; k < g_StateSpaceModel_Demo.Nx; ++k) {
            xhat[k] = g_StateSpaceModel_Demo.EstimatedState[k];
        }
        return;
    }

    if (std::strcmp(cmd, "terminate") == 0) {
        StateSpaceModel_Demo_EstimatorPortTermination(&g_StateSpaceModel_Demo);
        if (nlhs > 0) plhs[0] = mxCreateLogicalScalar(true);
        return;
    }

    mexErrMsgIdAndTxt("EstimatorPortN:unknownCmd",
                      "Unknown command '%s'. Use 'initialize' | 'estimate' | 'terminate'.", cmd);
}
