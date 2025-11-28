% ===========================================================================
% Filename: M_SO_Demo.m
% Description:
%
% A demo for demonstrating how to call estimator port in .so form on Linux.
% It uses the MEX wrapper Estimator/EstimatorPortN_MatlabSO.mexa64.
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

clear; close all; clc;

% 工程根目录（此文件所在目录）
projRoot = fileparts(fileparts(mfilename('fullpath')));
estDir   = fullfile(projRoot, 'Estimator');
outDir   = fullfile(projRoot, 'Output');

% 路径与目标
srcCpp   = fullfile(estDir, 'EstimatorPortN_MatlabSO.cpp');
outBase  = fullfile(outDir, 'EstimatorPortN_MatlabSO');        % 不带扩展名
outMex   = [outBase '.' mexext];                               % 带扩展名的完整输出文件

% 仅当 .mexa64 不存在时才编译（第一次）
if ~exist(outMex, 'file')
    % 如未配置过，先执行一次（只需手动一次）：  mex -setup C++
    mex( ...
        '-v', ...
        ['-I' estDir], ...                         % 头文件路径：Estimator/
        srcCpp, ...                                % 源文件：Estimator/EstimatorPortN_MatlabSO.cpp
        fullfile(outDir,'EstimatorPortN.so'), ...  % 直接传入 .so 的绝对/相对完整路径
        "LDFLAGS=$LDFLAGS -Wl,-rpath,'$ORIGIN'", ...  % 运行时在 mex 同目录找 .so
        '-output', outBase);                       % 输出到 Estimator/EstimatorPortN_MatlabSO.mexa64
end

addpath(fullfile(pwd, 'Output'));

% ========== 常量 ==========
INPUT_PATH  = fullfile('ObservationData','DoubleReflectorTrace','Trace1000.txt');
OUTPUT_PATH = fullfile('EstimationResult');
DATA_ROWS   = 1000;
READ_DATA_COLUMNS  = 3;
WRITE_DATA_COLUMNS = 5;
Observation_Dimension = 2;
State_Dimension       = 4;

% ========== 读取数据 ==========
data = dlmread(INPUT_PATH);
if size(data,1) ~= DATA_ROWS || size(data,2) ~= READ_DATA_COLUMNS
    error('Data size mismatch.');
end
ReadFileData = data;

% ========== 初始化 ==========
ok = EstimatorPortN_MatlabSO('initialize');
if ~ok, error('Estimator initialization failed.'); end

EstimatedState = zeros(DATA_ROWS, State_Dimension);
WriteFileData  = zeros(DATA_ROWS, WRITE_DATA_COLUMNS);

% ========== 逐步估计 ==========
for i = 1:DATA_ROWS
    observation = ReadFileData(i, 2:3)';              % [z1; z2]
    tstamp      = ReadFileData(i, 1);       
    xhat = EstimatorPortN_MatlabSO('estimate', observation, tstamp); % Nx x 1
    EstimatedState(i,:) = xhat(:).';
    WriteFileData(i,:)  = [ReadFileData(i,1) xhat(:).'];
end

% ========== 结束 ==========
ok = EstimatorPortN_MatlabSO('terminate');
if ~ok, error('Estimator termination failed.'); end

% ========== 写结果 ==========
if ~exist(OUTPUT_PATH,'dir'), mkdir(OUTPUT_PATH); end
timestamp = datestr(now, 'yyyymmddHHMMSS');
outFile = fullfile(OUTPUT_PATH, ['EstimationResult_' timestamp '.txt']);
fid = fopen(outFile,'w');
for i = 1:DATA_ROWS
    fprintf(fid, '%.6f %.6f %.6f %.6f %.6f\n', WriteFileData(i,:));
end
fclose(fid);

disp(['Estimation data has been written to ', outFile]);
disp('Program terminated...');
