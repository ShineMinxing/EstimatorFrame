function StateSpaceModelN = Estimator3011_1(StateSpaceModelN)
% Estimator3011_1 — Regression-Kalman (自适应回归卡尔曼估计器)
%
% 作者与联系方式
%   光电所一室2020级  孙敏行
%   401435318@qq.com

    % ===== 固定参数 =====
    FAK_DtS = 500;
    m = StateSpaceModelN.Nx / StateSpaceModelN.Nz;         % AR 阶数 = block size
    L = 2*FAK_DtS + m;              % 历史缓冲长度（对齐你 C 里 2*DtS + m 的搬移方式）

    % ===== Matrix_Par 内存布局（一维数组）=====
    % hist:  StateSpaceModelN.Nz * L
    % A:     StateSpaceModelN.Nz * (m*m)
    % B:     StateSpaceModelN.Nz * m
    % w:     StateSpaceModelN.Nz * m
    base_hist = 1;
    sz_hist   = StateSpaceModelN.Nz * L;
    base_A    = base_hist + sz_hist;
    sz_A      = StateSpaceModelN.Nz * (m*m);
    base_B    = base_A + sz_A;
    sz_B      = StateSpaceModelN.Nz * m;
    base_w    = base_B + sz_B;
    sz_w      = StateSpaceModelN.Nz * m;

    needed = base_w + sz_w - 1;
    StateSpaceModelN.Matrix_Par(needed,1) = 0;   % 扩容（必要的固定动作）

    % ===== 内部计数/标志（用 Int_Par）=====
    % DtSN: 当前已写入的样本序号（从 0 开始累加）
    % Flag: 是否进入滚动更新阶段（=1 表示 A,B 可以减旧加新）
    DtSN = StateSpaceModelN.Int_Par(10);
    Flag = StateSpaceModelN.Int_Par(11);

    % ===== 当前观测 =====
    z = StateSpaceModelN.CurrentObservation(:);

    % ===== 写入历史缓冲（按 DtSN 累加写入，不做复杂检查）=====
    % 1-based 下标：t = DtSN+1
    t = DtSN + 1;
    for k = 1:StateSpaceModelN.Nz
        StateSpaceModelN.Matrix_Par(base_hist + (k-1)*L + (t-1)) = z(k);
    end

    % ===== 取/写 A,B,w（按维度分块）=====
    % helper: 取第 k 个维度的 A(mxm), B(mx1)
    % A_k 存在 base_A + (k-1)*m*m
    % B_k 存在 base_B + (k-1)*m
    % w_k 存在 base_w + (k-1)*m

    % ===== 当样本达到 (FAK_DtS + m - 1) 时，初始化一次 A,B,w =====
    if DtSN == (FAK_DtS + m - 1)
        Flag = 1;

        for k = 1:StateSpaceModelN.Nz
            A = zeros(m,m);
            B = zeros(m,1);

            % 用 n = m+1 ... m+FAK_DtS 构建 LS
            % x_n = [z(n-1), z(n-2), ... z(n-m)]^T
            % y_n = z(n)
            for n = (m+1) : (m+FAK_DtS)
                xn = zeros(m,1);
                for j = 1:m
                    xn(j) = StateSpaceModelN.Matrix_Par(base_hist + (k-1)*L + (n-j-1));
                end
                yn = StateSpaceModelN.Matrix_Par(base_hist + (k-1)*L + (n-1));
                A = A + (xn * xn.');
                B = B + (xn * yn);
            end

            w = A \ B;  % 等价于 C 里的 inverse(A)*B（数值上更稳）

            % 写回 Matrix_Par
            StateSpaceModelN.Matrix_Par(base_A + (k-1)*m*m : base_A + k*m*m - 1) = A(:);
            StateSpaceModelN.Matrix_Par(base_B + (k-1)*m   : base_B + k*m   - 1) = B(:);
            StateSpaceModelN.Matrix_Par(base_w + (k-1)*m   : base_w + k*m   - 1) = w(:);
        end
    end

    % ===== 进入滚动阶段：A,B 减旧加新，然后更新 w =====
    if Flag == 1 && DtSN > (FAK_DtS + m - 1)
        % 旧样本对应的输出索引 n_old = DtSN - FAK_DtS
        % 新样本对应的输出索引 n_new = DtSN
        n_old = DtSN - FAK_DtS;
        n_new = DtSN;

        for k = 1:StateSpaceModelN.Nz
            A = reshape(StateSpaceModelN.Matrix_Par(base_A + (k-1)*m*m : base_A + k*m*m - 1), m, m);
            B = StateSpaceModelN.Matrix_Par(base_B + (k-1)*m : base_B + k*m - 1);

            % x_old = [z(n_old-1)...z(n_old-m)]，y_old = z(n_old)
            x_old = zeros(m,1);
            for j = 1:m
                x_old(j) = StateSpaceModelN.Matrix_Par(base_hist + (k-1)*L + (n_old-j));
            end
            y_old = StateSpaceModelN.Matrix_Par(base_hist + (k-1)*L + (n_old));

            % x_new = [z(n_new-1)...z(n_new-m)]，y_new = z(n_new)
            x_new = zeros(m,1);
            for j = 1:m
                x_new(j) = StateSpaceModelN.Matrix_Par(base_hist + (k-1)*L + (n_new-j));
            end
            y_new = StateSpaceModelN.Matrix_Par(base_hist + (k-1)*L + (n_new));

            % A,B 滚动更新（对应你 C 里的 “A减旧+加新”，“B减旧+加新”）:contentReference[oaicite:1]{index=1}
            A = A - (x_old*x_old.') + (x_new*x_new.');
            B = B - (x_old*y_old)   + (x_new*y_new);

            w = A \ B;

            StateSpaceModelN.Matrix_Par(base_A + (k-1)*m*m : base_A + k*m*m - 1) = A(:);
            StateSpaceModelN.Matrix_Par(base_B + (k-1)*m   : base_B + k*m   - 1) = B(:);
            StateSpaceModelN.Matrix_Par(base_w + (k-1)*m   : base_w + k*m   - 1) = w(:);
        end
    end

    % ===== 用回归得到的 w 调整 F（每个 block 一个 companion 结构）=====
    % F 的每个 block：
    %   第一行：w'（长度 m）
    %   次对角：1（实现 shift）
    F = StateSpaceModelN.Matrix_F;
    if isempty(F)
        F = eye(StateSpaceModelN.Nx);
    else
        F(:) = 0;
    end

    for k = 1:StateSpaceModelN.Nz
        w = StateSpaceModelN.Matrix_Par(base_w + (k-1)*m : base_w + k*m - 1);
        r0 = 1 + (k-1)*m;
        c0 = 1 + (k-1)*m;

        % 第一行写 w'
        F(r0, c0:c0+m-1) = w(:).';

        % shift：2..m 行
        for i = 2:m
            F(r0 + i - 1, c0 + i - 2) = 1.0;
        end
    end

    StateSpaceModelN.Matrix_F = F;

    % ===== 标准 Kalman：predict -> update =====
    X = StateSpaceModelN.EstimatedState;
    P = StateSpaceModelN.Matrix_P;
    H = StateSpaceModelN.Matrix_H;
    Q = StateSpaceModelN.Matrix_Q;
    R = StateSpaceModelN.Matrix_R;

    % predict
    Xpre = F * X;
    Ppre = F * P * F.' + Q;

    % update
    S = H * Ppre * H.' + R;
    K = (Ppre * H.') / S;
    innov = z - H * Xpre;

    X = Xpre + K * innov;
    P = (eye(StateSpaceModelN.Nx) - K*H) * Ppre;

    StateSpaceModelN.EstimatedState = X;
    StateSpaceModelN.Matrix_P = P;

    % ===== 预测 PredictStep 步（用同一个 F）=====
    Xp = X;
    if StateSpaceModelN.PredictStep ~= 0
        for i = 1:StateSpaceModelN.PredictStep
            Xp = F * Xp;
        end
    end
    StateSpaceModelN.PredictedState = Xp;
    StateSpaceModelN.PredictedObservation = H * Xp;

    % ===== 更新计数/搬移（对齐 C 里超过 2*DtS+m 就整体左移 DtS）=====
    DtSN = DtSN + 1;
    if DtSN >= (2*FAK_DtS + m)
        DtSN = DtSN - FAK_DtS;

        % 把 hist 的 [FAK_DtS+1 ... FAK_DtS+FAK_DtS+m] 搬到前面
        % 对每个维度独立搬
        for k = 1:StateSpaceModelN.Nz
            src0 = base_hist + (k-1)*L + FAK_DtS;
            dst0 = base_hist + (k-1)*L;
            StateSpaceModelN.Matrix_Par(dst0 + (0:(FAK_DtS+m-1))) = ...
                StateSpaceModelN.Matrix_Par(src0 + (0:(FAK_DtS+m-1)));
            % 清零尾部（可选，但和你 C 的习惯一致）
            StateSpaceModelN.Matrix_Par(dst0 + (FAK_DtS+m):(L-1)) = 0;
        end
    end

    StateSpaceModelN.Int_Par(10) = DtSN;
    StateSpaceModelN.Int_Par(11) = Flag;
end
