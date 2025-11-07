function StateSpaceModelN = StateSpaceModel2(StateSpaceModelN)
    % 初始化结构体中的变量
    StateSpaceModelN.PortName = '2D Quadratic Drag Motion Model with Added Diff Function for EKF';
    StateSpaceModelN.PortIntroduction = 'For Reference';

    StateSpaceModelN.Nx = 4;
    StateSpaceModelN.Nz = 2;
    StateSpaceModelN.PredictStep = 3;
    StateSpaceModelN.Intervel = 0.005;
    StateSpaceModelN.PredictTime = StateSpaceModelN.PredictStep * StateSpaceModelN.Intervel;

    StateSpaceModelN.EstimatedState = [1; 10; 1; -10];
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

    StateSpaceModelN.Int_Par = zeros(100,1);
    StateSpaceModelN.Double_Par = zeros(100,1);
    StateSpaceModelN.Double_Par(1:4) = [100.0 0.1 0.0 0.0];  % 分别是 质量 二次阻力系数 x轴外部加速度 y轴外部加速度


    % 定义结构体中的函数句柄
    StateSpaceModelN.StateTransitionEquation = @(In_State, StateSpaceModelN) StateSpaceModel2StateTransitionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.StateTransitionDiffEquation = @(In_State, StateSpaceModelN) StateSpaceModel2StateTransitionDiffFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.ObservationEquation = @(In_State, StateSpaceModelN) StateSpaceModel2ObservationFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.ObservationDiffEquation = @(In_State, StateSpaceModelN) StateSpaceModel2ObservationDiffFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.PredictionEquation = @(In_State, StateSpaceModelN) StateSpaceModel2PredictionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.EstimatorPort = @(StateSpaceModelN) StateSpaceModel2EstimatorPort(StateSpaceModelN);
    StateSpaceModelN.EstimatorPortTermination = @() StateSpaceModel2EstimatorPortTermination();
end


% 定义各个函数的实现
function [Out_State, StateSpaceModelN] = StateSpaceModel2StateTransitionFunction(In_State, StateSpaceModelN)
    % 二次阻力：x=[x vx y vy]^T，h=[x y]^T
    Out_State = zeros(StateSpaceModelN.Nx,1);
    dt = StateSpaceModelN.Intervel;
    v = sqrt(In_State(2)*In_State(2) + In_State(4)*In_State(4) + 1e-12);
    Out_State(1) = In_State(1) + dt * In_State(2);
    Out_State(2) = In_State(2) + dt * ( StateSpaceModelN.Double_Par(3) ...
                  - (StateSpaceModelN.Double_Par(2)/StateSpaceModelN.Double_Par(1)) * v * In_State(2) );
    Out_State(3) = In_State(3) + dt * In_State(4);
    Out_State(4) = In_State(4) + dt * ( StateSpaceModelN.Double_Par(4) ...
                  - (StateSpaceModelN.Double_Par(2)/StateSpaceModelN.Double_Par(1)) * v * In_State(4) );
end


function [Out_State, StateSpaceModelN] = StateSpaceModel2StateTransitionDiffFunction(In_State, StateSpaceModelN)
    % F = ∂f/∂x（在 In_State 处）
    dt = StateSpaceModelN.Intervel;
    vx = In_State(2); vy = In_State(4);
    v  = sqrt(vx*vx + vy*vy + 1e-12);
    Out_State = [ ...
        1, dt, 0,  0; ...
        0, 1 - dt*(StateSpaceModelN.Double_Par(2)/StateSpaceModelN.Double_Par(1)) * ((2*vx*vx + vy*vy)/v), 0, - dt*(StateSpaceModelN.Double_Par(2)/StateSpaceModelN.Double_Par(1)) * ((vx*vy)/v); ...
        0, 0,  1, dt; ...
        0, - dt*(StateSpaceModelN.Double_Par(2)/StateSpaceModelN.Double_Par(1)) * ((vx*vy)/v), 0, 1 - dt*(StateSpaceModelN.Double_Par(2)/StateSpaceModelN.Double_Par(1)) * ((vx*vx + 2*vy*vy)/v) ...
    ];
end


function [Out_Observation, StateSpaceModelN] = StateSpaceModel2ObservationFunction(In_State, StateSpaceModelN)
    % h(x) = [x; y]
    Out_Observation = zeros(StateSpaceModelN.Nz,1);
    Out_Observation(1) = In_State(1);
    Out_Observation(2) = In_State(3);
end


function [Out_Observation, StateSpaceModelN] = StateSpaceModel2ObservationDiffFunction(In_State, StateSpaceModelN)
    % H = ∂h/∂x
    Out_Observation = [ ...
        1, 0, 0, 0; ...
        0, 0, 1, 0 ...
    ];
end


function [Out_PredictedState, StateSpaceModelN] = StateSpaceModel2PredictionFunction(In_State, StateSpaceModelN)
    % 前视预测（同二次阻力模型），用 PredictTime
    Out_PredictedState = zeros(StateSpaceModelN.Nx,1);
    T = StateSpaceModelN.PredictTime;
    v = sqrt(In_State(2)*In_State(2) + In_State(4)*In_State(4) + 1e-12);
    Out_PredictedState(1) = In_State(1) + T * In_State(2);
    Out_PredictedState(2) = In_State(2) + T * ( StateSpaceModelN.Double_Par(3) ...
                           - (StateSpaceModelN.Double_Par(2)/StateSpaceModelN.Double_Par(1)) * v * In_State(2) );
    Out_PredictedState(3) = In_State(3) + T * In_State(4);
    Out_PredictedState(4) = In_State(4) + T * ( StateSpaceModelN.Double_Par(4) ...
                           - (StateSpaceModelN.Double_Par(2)/StateSpaceModelN.Double_Par(1)) * v * In_State(4) );
end



function StateSpaceModelN = StateSpaceModel2EstimatorPort(StateSpaceModelN)
    StateSpaceModelN = Estimator3004(StateSpaceModelN);
end

function StateSpaceModel2EstimatorPortTermination()
    fprintf('EstimatorPort terminated.\n');
end