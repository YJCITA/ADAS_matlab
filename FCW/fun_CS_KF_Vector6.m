% 一步EKF
function [Xk, Pk] = fun_CS_KF_Vector6(Xk_1, Pk_1, z, Q, R, F, U, H, alpha, a_max, T)
    x_dimension = length(Xk_1);
    In = eye(x_dimension);
    %
    E_a = [Xk_1(3), Xk_1(6)]';
    [Xk_predict Q_new] = fun_CS_model_Vector6(Xk_1, E_a, alpha, a_max, T, F, U);
    
    Xk_predict1 = F*Xk_1;               % 1.状态一步预测    
    Pk_predict = F*Pk_1*F' + Q_new;        % 2.预测误差协方差阵
    S = H*Pk_predict*H' + R;           % 信息协方差阵
    Kk = Pk_predict*H'*(S^-1);          %3. 增益矩阵
    Z_predict = H*Xk_predict;    
    z_info = z - Z_predict;
    tt1 = Kk*z_info;
    Xk = Xk_predict + Kk*z_info;% 4.状态估计
    Pk = (In - Kk*H)*Pk_predict;     % 5.协方差估计
end