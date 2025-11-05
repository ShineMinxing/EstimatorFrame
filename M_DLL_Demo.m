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

% Only compile once. Annotat this line after generated M_DLL_Demo.mexw64 etc.
mex -I'Estimator' Estimator\EstimatorPortN_MatlabDLL.cpp 'Estimator\EstimatorPortN.lib' -outdir 'Estimator'
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

clear all;
close all;
clc;

% 添加包含 DLL 文件的路径
addpath(fullfile(fileparts(mfilename('fullpath')), 'Estimator'));

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
