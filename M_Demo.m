%% M_Demo.m - MATLAB Script for Estimator Frame Demo

close all; clear; clc;

% 设置估计算法路径
addpath('Estimator/M_Estimators');

% 宏配置
DATA_ROWS = 3000;
READ_DATA_COLUMNS = 4;
WRITE_DATA_COLUMNS = 11;
State_Dimension = 4;
% 分配空间
ReadFileData = zeros(DATA_ROWS, READ_DATA_COLUMNS);
WriteFileData = zeros(DATA_ROWS, WRITE_DATA_COLUMNS);
EstimatedState = zeros(DATA_ROWS, State_Dimension);

StateColumns = [2,3];
ObserColumns = [4,5];

% 生成观测数据
StateSpaceModel1_ = struct();
StateSpaceModel1_ = StateSpaceModel1(StateSpaceModel1_);  %StateSpaceModel1_ 调用初始化函数来初始化全局结构体
ReadFileData(1, StateColumns) = StateSpaceModel1_.Matrix_H *  StateSpaceModel1_.EstimatedState;
ReadFileData(1, ObserColumns) = StateSpaceModel1_.Matrix_H *  StateSpaceModel1_.EstimatedState;
for i = 2:DATA_ROWS
    StateSpaceModel1_ = NonlinearTrajectoryGeneration1(StateSpaceModel1_);
    ReadFileData(i, 1) = ReadFileData(i - 1, 1) + StateSpaceModel1_.Intervel;
    ReadFileData(i, StateColumns) = StateSpaceModel1_.EstimatedState([1,3])';
    ReadFileData(i, ObserColumns) = StateSpaceModel1_.CurrentObservation';
end
ReadFileData(:, [2, 3]) =  [repmat( ReadFileData(1, [2, 3]), StateSpaceModel1_.PredictStep, 1 ); ReadFileData(1 : (end - StateSpaceModel1_.PredictStep), [2, 3])];
WriteFileData(:, 1) = ReadFileData(:, 1); % 时间戳


% 初始化估计器结构体1
StateSpaceModel1_ = struct();
StateSpaceModel1_ = StateSpaceModel1(StateSpaceModel1_);  %StateSpaceModel1_ 调用初始化函数来初始化全局结构体
% 使用 StateSpaceModel1_ 进行估计
for i = 1:DATA_ROWS
    % 提取观测数据
    StateSpaceModel1_.CurrentObservation = ReadFileData(i, ObserColumns)';
    % 使用估计器进行估计并更新结构体
    StateSpaceModel1_ = StateSpaceModel1_.EstimatorPort(StateSpaceModel1_);
    % 构建写入数据
    WriteFileData(i, 2:5) = StateSpaceModel1_.PredictedState; % 状态
end


% 写入输出文件
timestamp = datetime('now', 'Format', 'yyyyMMddHHmmss');
fullOutputFilePath = fullfile(['EstimationResult/EstimationResult_', char(timestamp), '.txt']);

fileID = fopen(fullOutputFilePath, 'w');
if fileID == -1
    error('Unable to create file: %s', fullOutputFilePath);
end
[dataRows, dataCols] = size(WriteFileData);
for i = 1:dataRows
    fprintf(fileID, '%.6f ', WriteFileData(i, 1:dataCols));
    fprintf(fileID, '\n');
end
fclose(fileID);
fprintf('Estimation data has been written to %s\n', fullOutputFilePath);

fprintf('Program terminated...\n');
