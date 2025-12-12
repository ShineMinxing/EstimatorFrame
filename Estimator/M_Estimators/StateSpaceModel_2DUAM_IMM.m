function StateSpaceModelN = StateSpaceModel_2DUAM_IMM(StateSpaceModelN)
    % 2D Uniform Acceleration Model + IMM Wrapper
    % 状态: x = [x; vx; ax; y; vy; ay]

    StateSpaceModelN.PortName        = 'IMM-2D UAM';
    StateSpaceModelN.PortIntroduction= 'For Reference';

    StateSpaceModelN.Nx          = 6;
    StateSpaceModelN.Nz          = 2;
    StateSpaceModelN.PredictStep = 3;
    StateSpaceModelN.Intervel    = 0.005;
    StateSpaceModelN.PredictTime = StateSpaceModelN.PredictStep * StateSpaceModelN.Intervel;

    % 初始状态: 位置+速度+加速度
    StateSpaceModelN.EstimatedState       = [1; 10; 0; 1; -10; 0];
    StateSpaceModelN.PredictedState       = zeros(StateSpaceModelN.Nx, 1);
    StateSpaceModelN.CurrentObservation   = zeros(StateSpaceModelN.Nz, 1);
    StateSpaceModelN.PredictedObservation = zeros(StateSpaceModelN.Nz, 1);

    % ===== 匀加速度线性 F (常矩阵) =====
    dt = StateSpaceModelN.Intervel;
    StateSpaceModelN.Matrix_F = eye(StateSpaceModelN.Nx);
    % x 更新
    StateSpaceModelN.Matrix_F(1,2) = dt;
    StateSpaceModelN.Matrix_F(1,3) = 0.5 * dt^2;
    StateSpaceModelN.Matrix_F(2,3) = dt;
    % y 更新
    StateSpaceModelN.Matrix_F(4,5) = dt;
    StateSpaceModelN.Matrix_F(4,6) = 0.5 * dt^2;
    StateSpaceModelN.Matrix_F(5,6) = dt;
    % ax, ay 视为常值状态: ax(k+1)=ax, ay(k+1)=ay

    StateSpaceModelN.Matrix_G = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_B = zeros(StateSpaceModelN.Nx,1);

    % 观测: z = [x; y]
    StateSpaceModelN.Matrix_H = zeros(StateSpaceModelN.Nz, StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_H(1,1) = 1;   % x
    StateSpaceModelN.Matrix_H(2,4) = 1;   % y

    StateSpaceModelN.Matrix_P = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_Q = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_R = eye(StateSpaceModelN.Nz);

    StateSpaceModelN.Int_Par    = zeros(100,1);
    StateSpaceModelN.Double_Par = zeros(100,1);
    % 仍然保留参数位，方便以后扩展（此处匀加速度未用到）
    StateSpaceModelN.Double_Par(1:4) = [100.0; 0.1; 0.0; 0.0];
    StateSpaceModelN.Matrix_Par = zeros(100,1);

    % ===== 函数句柄（统统匀加速度线性模型） =====
    StateSpaceModelN.StateTransitionEquation     =  @(In_State, StateSpaceModelN) StateSpaceModel_2DUAM_IMMStateTransitionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.ObservationEquation         =  @(In_State, StateSpaceModelN) StateSpaceModel_2DUAM_IMMObservationFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.PredictionEquation          =  @(In_State, StateSpaceModelN) StateSpaceModel_2DUAM_IMMPredictionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.EstimatorPort               =  @(StateSpaceModelN) StateSpaceModel_2DUAM_IMMEstimatorPort(StateSpaceModelN);
    StateSpaceModelN.EstimatorPortTermination    =  @(StateSpaceModelN) StateSpaceModel_2DUAM_IMMEstimatorPortTermination(StateSpaceModelN);

    StateSpaceModelN.IMM_ModeNames = {'SubModel1','SubModel2','SubModel3'};
    StateSpaceModelN.IMM_Mn  = numel(StateSpaceModelN.IMM_ModeNames);

    StateSpaceModelN.(StateSpaceModelN.IMM_ModeNames{1}) = struct();
    StateSpaceModelN.(StateSpaceModelN.IMM_ModeNames{1}) = ...
        StateSpaceModel_2DSinM(StateSpaceModelN.(StateSpaceModelN.IMM_ModeNames{1}));

    StateSpaceModelN.(StateSpaceModelN.IMM_ModeNames{2}) = struct();
    StateSpaceModelN.(StateSpaceModelN.IMM_ModeNames{2}) = ...
        StateSpaceModel_2DUAM(StateSpaceModelN.(StateSpaceModelN.IMM_ModeNames{2}));

    StateSpaceModelN.(StateSpaceModelN.IMM_ModeNames{3}) = struct();
    StateSpaceModelN.(StateSpaceModelN.IMM_ModeNames{3}) = ...
        StateSpaceModel_2DDMM(StateSpaceModelN.(StateSpaceModelN.IMM_ModeNames{3}));

    % 模式转移矩阵 & 初始概率
    StateSpaceModelN.IMM_Ptrans   = [0.90 0.05 0.05;
                                     0.05 0.90 0.05;
                                     0.05 0.05 0.90];
    StateSpaceModelN.IMM_ModeProb = [0.4; 0.3; 0.3];
end

function [Out_State, StateSpaceModelN] = StateSpaceModel_2DUAM_IMMStateTransitionFunction(In_State, StateSpaceModelN)
    Out_State = StateSpaceModelN.Matrix_F * In_State;
end

function [Out_Observation, StateSpaceModelN] = StateSpaceModel_2DUAM_IMMObservationFunction(In_State, StateSpaceModelN)
    Out_Observation = StateSpaceModelN.Matrix_H * In_State;
end

function [Out_PredictedState, StateSpaceModelN] = StateSpaceModel_2DUAM_IMMPredictionFunction(In_State, StateSpaceModelN)
    T = StateSpaceModelN.PredictTime;
    Matrix_F = eye(StateSpaceModelN.Nx);
    Matrix_F(1,2) = T;
    Matrix_F(1,3) = 0.5 * T^2;
    Matrix_F(2,3) = T;
    Matrix_F(4,5) = T;
    Matrix_F(4,6) = 0.5 * T^2;
    Matrix_F(5,6) = T;

    Out_PredictedState = Matrix_F * In_State;
end


% ================== 估计器入口：IMM 顶层 ==================
function StateSpaceModelN = StateSpaceModel_2DUAM_IMMEstimatorPort(StateSpaceModelN)
    StateSpaceModelN = Estimator3010(StateSpaceModelN);
end


function StateSpaceModel_2DUAM_IMMEstimatorPortTermination(StateSpaceModelN)
    fprintf('StateSpaceModel_2DUAM_IMM EstimatorPort terminated.\n');
end
