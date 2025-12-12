function StateSpaceModelN = Estimator3011(StateSpaceModelN)
% Estimator3011 — Sliding-Window Linear Regression (固定参数版)
%
% 作者与联系方式
%   光电所一室2020级  孙敏行
%   401435318@qq.com

    Nz = StateSpaceModelN.Nz;
    Nx = StateSpaceModelN.Nx;
    dt = StateSpaceModelN.Intervel;

    % ===== 固定滑窗长度 =====
    WIN = 50;

    % ===== 预测步数（step=0 时就按 1 步）=====
    step = StateSpaceModelN.PredictStep;
    if step == 0
        step = 1;
    end

    % ===== 当前观测（不做长度修正）=====
    z = StateSpaceModelN.CurrentObservation(:);

    % ===== ring buffer in Matrix_Par（一维数组使用）=====
    % 布局：
    %   Matrix_Par(1 : WIN*Nz)          -> z_buf
    %   Int_Par(10)                     -> idx  (1..WIN)
    %   Int_Par(11)                     -> count(0..WIN)
    base_z = 1;
    needed = base_z + WIN*Nz - 1;
    StateSpaceModelN.Matrix_Par(needed, 1) = 0;  % 扩容（若已足够则无影响）

    idx   = StateSpaceModelN.Int_Par(10);
    if idx == 0
        idx = 1;
    end
    count = StateSpaceModelN.Int_Par(11);

    % 写入第 idx 个槽
    for k = 1:Nz
        StateSpaceModelN.Matrix_Par(base_z + (k-1)*WIN + idx - 1) = z(k);
    end

    % advance idx/count（不做越界检查：按你的风格保持简单，但仍保证环形）
    idx = idx + 1;
    if idx > WIN
        idx = 1;
    end
    if count < WIN
        count = count + 1;
    end
    StateSpaceModelN.Int_Par(10) = idx;
    StateSpaceModelN.Int_Par(11) = count;

    % ===== pos 索引规则：Nx = x*Nz 时，用 1,1+x,... =====
    x = Nx / Nz;   % 这里按你规则默认它就是整数
    pos_idx = zeros(Nz,1);
    vel_idx = zeros(Nz,1);

    for k = 1:Nz
        pos_idx(k) = 1 + (k-1)*x;
        if pos_idx(k) + 1 <= Nx
            vel_idx(k) = pos_idx(k) + 1;
        else
            vel_idx(k) = 0;
        end
    end

    % ===== 历史不足：直接把当前观测写入位置，速度置 0 =====
    if count < 2
        xhat = StateSpaceModelN.EstimatedState(:);
        if numel(xhat) ~= Nx
            xhat = zeros(Nx,1);
        end
        for k = 1:Nz
            p = pos_idx(k);
            xhat(p) = z(k);
            if vel_idx(k) ~= 0
                xhat(vel_idx(k)) = 0;
            end
        end
        StateSpaceModelN.EstimatedState = xhat;
        StateSpaceModelN.PredictedState = xhat;
        StateSpaceModelN.PredictedObservation = StateSpaceModelN.Matrix_H * xhat;
        return;
    end

    % ===== 取出最近 count 个样本（按时间顺序）=====
    newest = idx - 1;
    if newest < 1
        newest = WIN;
    end
    ord = newest - (count-1) : newest;
    ord = mod(ord-1, WIN) + 1;   % 1..WIN

    n  = (1:count).';
    n0 = mean(n);
    nc = n - n0;
    denom = nc.' * nc;

    n_now  = double(count);          % 窗口末端
    n_pred = double(count + step);   % 预测点

    % ===== 对每个维度做一阶回归 =====
    pos_now  = zeros(Nz,1);
    pos_pred = zeros(Nz,1);
    vel_now  = zeros(Nz,1);   % 物理速度（/dt）
    vel_pred = zeros(Nz,1);

    for k = 1:Nz
        y = StateSpaceModelN.Matrix_Par(base_z + (k-1)*WIN + ord - 1);
        y = y(:);
        y0 = mean(y);

        a = (nc.' * (y - y0)) / denom;   % slope: unit/sample
        b = y0;                          % centered intercept

        pos_now(k)  = a * (n_now  - n0) + b;
        pos_pred(k) = a * (n_pred - n0) + b;

        vel_now(k)  = a / dt;
        vel_pred(k) = a / dt;
    end

    % ===== 写回状态：Estimated=当前，Predicted=未来 =====
    xhat_now = StateSpaceModelN.EstimatedState(:);
    if numel(xhat_now) ~= Nx
        xhat_now = zeros(Nx,1);
    end
    xhat_pred = xhat_now;

    for k = 1:Nz
        p = pos_idx(k);
        xhat_now(p)  = pos_now(k);
        xhat_pred(p) = pos_pred(k);

        v = vel_idx(k);
        if v ~= 0
            xhat_now(v)  = vel_now(k);
            xhat_pred(v) = vel_pred(k);
        end
    end

    StateSpaceModelN.EstimatedState = xhat_now;
    StateSpaceModelN.PredictedState = xhat_pred;

    StateSpaceModelN.PredictedObservation = StateSpaceModelN.Matrix_H * xhat_pred;
end
