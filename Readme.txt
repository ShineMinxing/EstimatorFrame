估计算法接口测试程序

编译之前：
1）检查.vscode\c_cpp_properties.json 和.vscode\XXXXX.bat中的依赖，和你的电脑的各种软件的版本名和路径是否符合。
2）目前的配置适用于在windows中进行编译。
3）请确保终端的路径在工程路径 XXXX\EstimatorFrame> 下。
4）用于测试的观测数据为ObservationData\DoubleReflectorTrace\Trace1000.txt，可自行更改为其他的。

开始测试

1. 测试C程序
    1.1ctrl+shift+B ———— build C Demo, 
    将会执行tasks.json中的第二小节，跳转运行.vscode\build_c_demo.bat（后续类似，不再赘述）。
    该方法以C_Demo.c为主程序，编译Estimator\*.c，Estimator\C_Estimators\*.c，Estimator\Cpp_Estimators\*.cpp，最终生成C_Demo.exe。
    生成的杂七杂八文件，被转移到了Temp下（后续类似，不再赘述）。
    1.2 接下来ctrl+·打开终端，输入.\C_Demo.exe，将会看到
    “Tested Estimator v0.00 is initialized
    Unscented Kalman estimator is initialized
    EstimatorPort terminated.
    StateSpaceModel1_ finished...
    Estimation data has been written to EstimationResult\EstimationResult_2024XXXXXXXXXX.txt
    Program terminated...”
    1.3 生成的txt文件在EstimationResult中查看。

2. 更换估计器进行测试
    2.1 打开Estimator\StateSpaceModel1.c
    2.2 ctrl+F 查找所有 1001，替换为1002（c语言无迹卡尔曼）/2001（cpp语言线性卡尔曼）/2002（cpp语言无迹卡尔曼）。
    2.3 保存文件，之后ctrl+shift+B ———— build C Demo, 
    2.4 编译完成后，运行“.\C_Demo.exe”，查看使用不同估计器获得的EstimationResult\EstimationResult_2024XXXXXXXXXX.txt。

3. 修改状态空间模型进行测试
    3.1 打开Estimator\StateSpaceModel1.c，随便改点参数。
    建议把
    StateSpaceModel1_Initialization(struct EstimatorPortN *estimator);
    StateSpaceModel1_EstimatorPort(double *In_Observation, double *Out_State, struct EstimatorPortN *estimator);
    StateSpaceModel1_EstimatorPortTermination(struct EstimatorPortN *estimator);
    中的 +estimator->Intervel   +estimator->Intervel   +estimator->PredictTime 改为减。
    把
    double F[StateSpaceModel1_NX*StateSpaceModel1_NX] = {\
        1,StateSpaceModel1_Interval,0,0,\
        0,1,0,0,\
        0,0,1,StateSpaceModel1_Interval,\
        0,0,0,1\
        };
    中的StateSpaceModel1_Interval 改为 -StateSpaceModel1_Interval。
    这样效果明显。
    3.2 保存文件，之后ctrl+shift+B ———— build C Demo。
    3.3 编译完成后，运行“.\C_Demo.exe”，查看使用不同估计器获得的EstimationResult\EstimationResult_2024XXXXXXXXXX.txt。

4. 构建新状态空间模型进行测试
    4.1 在Estimator/EstimatorPortN.h中新增
    EXPORT extern EstimatorPortN StateSpaceModel2_;
    EXPORT void StateSpaceModel2_Initialization(struct EstimatorPortN *estimator);
    EXPORT void StateSpaceModel2_EstimatorPort(double *In_Observation, double *Out_State, struct EstimatorPortN *estimator);
    EXPORT void StateSpaceModel2_EstimatorPortTermination(struct EstimatorPortN *estimator);
    4.2 复制Estimator\StateSpaceModel1.c到同路径，并改名为StateSpaceModel2.c。
    4.3 在StateSpaceModel2.c中搜索StateSpaceModel1_，全部替换为StateSpaceModel2_，共替换约36个。
    4.4 在C_Demo.c的
    // StateSpaceModel1_ based estimation
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // StateSpaceModel1_ based estimation
    下方，新增
    // StateSpaceModel2_ based estimation
    StateSpaceModel2_Initialization(&StateSpaceModel2_);
    for (int i = 0; i < DATA_ROWS; i++) {
        for (int j = 0; j < Observation_Dimension; j++) {
            EstimatorObservation[i][j] = ReadFileData[i][j+1];
        }
        StateSpaceModel2_EstimatorPort(EstimatorObservation[i], EstimatedState[i], &StateSpaceModel2_);
        WriteFileData[i][0] = ReadFileData[i][0];
        for (int j = 0; j < State_Dimension; j++) {
            WriteFileData[i][j+5] = EstimatedState[i][j];
        }
    }
    StateSpaceModel2_EstimatorPortTermination(&StateSpaceModel2_);
    printf("StateSpaceModel2_ finished...\n");
    // StateSpaceModel2_ based estimation
    注意除了将StateSpaceModel1_替换为StateSpaceModel2_，还要把导出的数组的WriteFileData[i][j+1]，替换为WriteFileData[i][j+5]。
    4.5 保存文件，之后ctrl+shift+B ———— build C Demo。
    4.6 编译完成后，运行“.\C_Demo.exe”，查看EstimationResult\EstimationResult_2024XXXXXXXXXX.txt。
    
5. 测试Cpp程序
    5.1 ctrl+shift+B ———— build Cpp Demo。
    该方法以Cpp_Demo.cpp为主程序，编译Estimator\*.c，Estimator\C_Estimators\*.c，Estimator\Cpp_Estimators\*.cpp，最终生成Cpp_Demo.exe。
    5.2 接下来ctrl+·打开终端，输入.\Cpp_Demo.exe。
    5.3 生成的txt文件在EstimationResult中查看。

6. 生成动态链接库
    6.1 ctrl+shift+B ———— build DLL, 
    该方法以Estimator\StateSpaceModel1.c为主程序，编译Estimator\C_Estimators\*.c，Estimator\Cpp_Estimators\*.cpp，最终生成Estimator\EstimatorPortN.dll, Estimator\EstimatorPortN.lib。

7. 测试C程序调用DLL
    7.1 ctrl+shift+B ———— build C DLL Demo, 
    该方法以C_DLL_Demo.c为主程序，调用"Estimator\EstimatorPortN.lib"，最终生成C_DLL_Demo.exe。
    7.2 接下来ctrl+·打开终端，输入.\C_DLL_Demo.exe
    7.3 生成的txt文件在EstimationResult中查看。
    
8. 测试Cpp程序调用DLL
    8.1 ctrl+shift+B ———— build Cpp DLL Demo, 
    该方法以Cpp_DLL_Demo.cpp为主程序，调用"Estimator\EstimatorPortN.lib"，最终生成Cpp_DLL_Demo.exe。
    8.2 接下来ctrl+·打开终端，输入.\Cpp_DLL_Demo.exe。
    8.3 生成的txt文件在EstimationResult中查看。

9. 测试Python程序调用DLL
    9.1 ctrl+·打开终端，输入python Python_DLL_Demo.py。
    9.2 生成的txt文件在EstimationResult中查看。

10. 测试Matlab程序调用DLL
    10.1 Matlab打开M_DLL_Demo.m。
    10.2 首次运行时取消注释 mex -I'Estimator' Estimator\EstimatorPortN_MatlabDLL.cpp 'Estimator\EstimatorPortN.lib' -outdir 'Estimator'，该行代码利用Estimator\EstimatorPortN_MatlabDLL.cpp和库文件，生成matlab识别dll的中间文件Estimator\EstimatorPortN_MatlabDLL.mexw64。后续运行时，可注释掉这行，不需要重复生成.mexw64。
    10.2 M_DLL_Demo.m运行完，生成的txt文件在EstimationResult中查看。

11. 测试Matlab程序
    11.1 Matlab打开M_Demo.m，并运行。此代码调用接口Estimator\M_Estimators\StateSpaceModel1.m，接口中使用Estimator\M_Estimators\Estimator3001.m进行估计。
    11.2 M_Demo.m运行完，生成的txt文件在EstimationResult中查看。

12. 测试Matlab程序的新状态空间方程
    12.1 复制Estimator\M_Estimators\StateSpaceModel1.m为Estimator\M_Estimators\StateSpaceModel2.m。
    12.2 将Estimator\M_Estimators\StateSpaceModel2.m中的所有StateSpaceModel1替换为StateSpaceModel2，共约11处。
    12.3 将Estimator\M_Estimators\StateSpaceModel2.m中的函数Estimator3001更改为Estimator3002
    12.4 在M_Demo.m的
        % 初始化估计器结构体
        StateSpaceModel1_ = struct();
        StateSpaceModel1_ = StateSpaceModel1(StateSpaceModel1_);  %StateSpaceModel1_ 调用初始化函数来初始化全局结构体
        % 使用 StateSpaceModel1_ 进行估计
        for i = 1:DATA_ROWS
            % 提取观测数据
            StateSpaceModel1_.CurrentObservation = ReadFileData(i, 2:3)';
            % 使用估计器进行估计并更新结构体
            StateSpaceModel1_ = StateSpaceModel1_.EstimatorPort(StateSpaceModel1_);
            % 构建写入数据
            WriteFileData(i, 1) = ReadFileData(i, 1); % 时间戳
            WriteFileData(i, 2:5) = StateSpaceModel1_.EstimatedState; % 状态
        end
        之后，增加
        % 初始化估计器结构体
        StateSpaceModel2_ = struct();
        StateSpaceModel2_ = StateSpaceModel2(StateSpaceModel2_);  %StateSpaceModel2_ 调用初始化函数来初始化全局结构体
        % 使用 StateSpaceModel2_ 进行估计
        for i = 1:DATA_ROWS
            % 提取观测数据
            StateSpaceModel2_.CurrentObservation = ReadFileData(i, 2:3)';
            % 使用估计器进行估计并更新结构体
            StateSpaceModel2_ = StateSpaceModel2_.EstimatorPort(StateSpaceModel2_);
            % 构建写入数据
            WriteFileData(i, 1) = ReadFileData(i, 1); % 时间戳
            WriteFileData(i, 6:9) = StateSpaceModel2_.EstimatedState; % 状态
        end
    12.4 运行M_Demo.m，生成的txt文件在EstimationResult中查看。
