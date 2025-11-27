%% M_Demo.m - MATLAB Script for Estimator Frame Demo

close all; clear; clc;

% 设置估计算法路径
addpath('Estimator/M_Estimators');

% 宏配置
DATA_ROWS = 3000;

% 生成观测数据
StateSpaceModel4_ = struct();
StateSpaceModel4_ = StateSpaceModel4(StateSpaceModel4_);  %StateSpaceModel4_ 调用初始化函数来初始化全局结构体

State_Dimension = StateSpaceModel4_.Nx;
READ_DATA_COLUMNS = State_Dimension;
WRITE_DATA_COLUMNS = 1+State_Dimension;
% 分配空间
ReadFileData = zeros(DATA_ROWS, READ_DATA_COLUMNS);
WriteFileData = zeros(DATA_ROWS, WRITE_DATA_COLUMNS);
EstimatedState = zeros(DATA_ROWS, State_Dimension);

StateColumns = [2:1+StateSpaceModel4_.Nz];
ObserColumns = [2+StateSpaceModel4_.Nz:1+2*StateSpaceModel4_.Nz];

ReadFileData(1, StateColumns) = StateSpaceModel4_.Matrix_H *  StateSpaceModel4_.EstimatedState;
ReadFileData(1, ObserColumns) = StateSpaceModel4_.Matrix_H *  StateSpaceModel4_.EstimatedState;
for i = 2:DATA_ROWS
    StateSpaceModel4_ = NonlinearTrajectoryGeneration(StateSpaceModel4_);
    ReadFileData(i, 1) = ReadFileData(i - 1, 1) + StateSpaceModel4_.Intervel;
    ReadFileData(i, StateColumns) = StateSpaceModel4_.EstimatedState([1:StateSpaceModel4_.Nx/StateSpaceModel4_.Nz:end])';
    ReadFileData(i, ObserColumns) = StateSpaceModel4_.CurrentObservation';
end
ReadFileData(:, StateColumns) =  [repmat( ReadFileData(1, StateColumns), StateSpaceModel4_.PredictStep, 1 ); ReadFileData(1 : (end - StateSpaceModel4_.PredictStep), StateColumns)];
WriteFileData(:, 1) = ReadFileData(:, 1); % 时间戳


% 初始化估计器结构体1
StateSpaceModel5_ = struct();
StateSpaceModel5_ = StateSpaceModel5(StateSpaceModel5_);  %StateSpaceModel5_ 调用初始化函数来初始化全局结构体
% 使用 StateSpaceModel5_ 进行估计
for i = 1:DATA_ROWS
    % 提取观测数据
    StateSpaceModel5_.CurrentObservation = ReadFileData(i, ObserColumns)';
    % 使用估计器进行估计并更新结构体
    StateSpaceModel5_ = StateSpaceModel5_.EstimatorPort(StateSpaceModel5_);
    % 构建写入数据
    WriteFileData(i, 2:(1+State_Dimension)) = StateSpaceModel5_.PredictedState; % 状态
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

figure(1)
grid on; hold on;
plot(ReadFileData(:,StateColumns(1)),ReadFileData(:,StateColumns(2)),"k")
plot(ReadFileData(:,ObserColumns(1)),ReadFileData(:,ObserColumns(2)),"r.")
plot(WriteFileData(:,2),WriteFileData(:,2+State_Dimension/length(ObserColumns)),"b")
legend("真实值","观测值","估计值")
