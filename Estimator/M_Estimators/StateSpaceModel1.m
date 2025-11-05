function StateSpaceModelN = StateSpaceModel1(StateSpaceModelN)
    % 初始化结构体中的变量
    StateSpaceModelN.PortName = 'Tested Model 1 v0.00';
    StateSpaceModelN.PortIntroduction = 'For Reference';

    StateSpaceModelN.Nx = 4;
    StateSpaceModelN.Nz = 2;
    StateSpaceModelN.PredictStep = 3;
    StateSpaceModelN.Intervel = 0.005;
    StateSpaceModelN.PredictTime = StateSpaceModelN.PredictStep * StateSpaceModelN.Intervel;

    StateSpaceModelN.EstimatedState = [1; 0; 1; 0];
    StateSpaceModelN.PredictedState = zeros(StateSpaceModelN.Nx, 1);
    StateSpaceModelN.CurrentObservation = zeros(StateSpaceModelN.Nz, 1);
    StateSpaceModelN.PredictedObservation = zeros(StateSpaceModelN.Nz, 1);
    StateSpaceModelN.Matrix_F = [
        1, StateSpaceModelN.Intervel, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, StateSpaceModelN.Intervel;
        0, 0, 0, 1
    ];
    StateSpaceModelN.Matrix_G = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_B = [0; 0; 0; 0];
    StateSpaceModelN.Matrix_H = [
        1, 0, 0, 0;
        0, 0, 1, 0
    ];
    StateSpaceModelN.Matrix_P = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_Q = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_R = eye(StateSpaceModelN.Nz);

    StateSpaceModelN.Int_Par = 1;
    StateSpaceModelN.Double_Par = 1;

    % 定义结构体中的函数句柄
    StateSpaceModelN.StateTransitionEquation = @(In_State, StateSpaceModelN) StateSpaceModel1StateTransitionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.ObservationEquation = @(In_State, StateSpaceModelN) StateSpaceModel1ObservationFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.PredictionEquation = @(In_State, StateSpaceModelN) StateSpaceModel1PredictionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.EstimatorPort = @(StateSpaceModelN) StateSpaceModel1EstimatorPort(StateSpaceModelN);
    StateSpaceModelN.EstimatorPortTermination = @() StateSpaceModel1EstimatorPortTermination();
end

% 定义各个函数的实现
function [Out_State, StateSpaceModelN] = StateSpaceModel1StateTransitionFunction(In_State, StateSpaceModelN)
    Out_State = zeros(StateSpaceModelN.Nx,1);
    Out_State(1) = In_State(1) + StateSpaceModelN.Intervel * In_State(2);
    Out_State(2) = In_State(2);
    Out_State(3) = In_State(3) + StateSpaceModelN.Intervel * In_State(4);
    Out_State(4) = In_State(4);
end

function [Out_Observation, StateSpaceModelN] = StateSpaceModel1ObservationFunction(In_State, StateSpaceModelN)
    Out_Observation = zeros(StateSpaceModelN.Nz,1);
    Out_Observation(1) = In_State(1);
    Out_Observation(2) = In_State(3);
end

function [Out_PredictedState, StateSpaceModelN] = StateSpaceModel1PredictionFunction(In_State, StateSpaceModelN)
    Out_PredictedState = zeros(StateSpaceModelN.Nx,1);
    Out_PredictedState(1) = In_State(1) + StateSpaceModelN.PredictTime * In_State(2);
    Out_PredictedState(2) = In_State(2);
    Out_PredictedState(3) = In_State(3) + StateSpaceModelN.PredictTime * In_State(4);
    Out_PredictedState(4) = In_State(4);
end

function StateSpaceModelN = StateSpaceModel1EstimatorPort(StateSpaceModelN)
    StateSpaceModelN = Estimator3001(StateSpaceModelN);
end

function StateSpaceModel1EstimatorPortTermination()
    fprintf('EstimatorPort terminated.\n');
end