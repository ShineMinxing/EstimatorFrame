% ===========================================================================
% Filename: M_DLL_Demo.m
% Description:
%
% A demo for demonstrating how to call estimator port in dll form.
%
% You need to obtain more information from Estimator\EstimatorPortN_MatlabDLL.cpp.
%
% Initial version: Minxing Sun
% Unit: UCAS, Institute of Optics And Electronics, Lab 1, Class of 2020
% Email: 401435318@qq.com
% Date: November 17, 2024
% 
% Updates:
% Unit:
% Email:
% Date:
% ===========================================================================

clear; close all; clc;

% Only compile once. Annotat this line after generated M_DLL_Demo.mexw64 etc.
% mex -I'Estimator' Estimator\EstimatorPortN_MatlabDLL.cpp 'Estimator\EstimatorPortN.lib' -outdir 'Estimator'
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% 工程根目录（此文件所在目录）
projRoot = fileparts(fileparts(mfilename('fullpath')));
estDir   = fullfile(projRoot, 'Estimator');
outDir   = fullfile(projRoot, 'Output');

% 路径与目标
srcCpp   = fullfile(estDir, 'EstimatorPortN_MatlabDLL.cpp');
outBase  = fullfile(outDir, 'EstimatorPortN_MatlabDLL');        % 不带扩展名
outMex   = [outBase '.' mexext];                                % 带扩展名的完整输出文件
libPath  = fullfile(outDir, 'EstimatorPortN.lib');              % 库文件完整路径

% 仅当 .mexw64 不存在时才编译（第一次）
if ~exist(outMex, 'file')
    % 如未配置过，先执行一次（只需手动一次）：  mex -setup C++
    mex( ...
        '-v', ...
        ['-I' estDir], ...                     % 头文件路径
        srcCpp, ...                            % 源文件
        libPath, ...                           % 静态库路径
        '-outdir', outDir, ...                 % 输出目录
        '-output', outBase);                   % 输出文件名（不带扩展名）
end

% 添加包含 DLL 文件的路径
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Estimator'));
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Output'));

% Define constants
INPUT_PATH = fullfile('ObservationData', 'DoubleReflectorTrace', 'Trace1000.txt');
OUTPUT_PATH = fullfile('EstimationResult');
DATA_ROWS = 1000;
READ_DATA_COLUMNS = 3;
WRITE_DATA_COLUMNS = 5;
Observation_Dimension = 2;
State_Dimension = 4;

% Read data from file
data = dlmread(INPUT_PATH);
if size(data, 1) ~= DATA_ROWS || size(data, 2) ~= READ_DATA_COLUMNS
    error('Data size mismatch.');
end

ReadFileData = data;

% Initialize the estimator
success = EstimatorPortN_MatlabDLL('initialize');
if ~success
    error('Estimator initialization failed.');
end

EstimatedState = zeros(DATA_ROWS, State_Dimension);
WriteFileData = zeros(DATA_ROWS, WRITE_DATA_COLUMNS);

for i = 1:DATA_ROWS
    % Prepare observation data
    observation = ReadFileData(i, 2:3)';
    
    % Call the estimation function
    estimatedState = EstimatorPortN_MatlabDLL('estimate', observation);
    
    % Store results
    EstimatedState(i, :) = estimatedState';
    WriteFileData(i, :) = [ReadFileData(i, 1), estimatedState'];
end

% Terminate the estimator
success = EstimatorPortN_MatlabDLL('terminate');
if ~success
    error('Estimator termination failed.');
end

% Write results to file
timestamp = datestr(now, 'yyyymmddHHMMSS');
outputFilePath = fullfile(OUTPUT_PATH, ['EstimationResult_', timestamp, '.txt']);
dlmwrite(outputFilePath, WriteFileData, 'precision', '%.6f');

disp(['Estimation data has been written to ', outputFilePath]);
disp('Program terminated...');
