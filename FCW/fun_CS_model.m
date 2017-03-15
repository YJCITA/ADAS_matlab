% CS运动模型
function [Xk_new Q] = fun_CS_model(Xk, E_a, alpha, a_max, T)
% Xk = [x x' x''];
% alpha: 机动频率
% E_a： 加速度平均值
% aX_max,aY_max : [-max, +max] 最大最小加速度
    acc_cur = Xk(3); % 当前估计的加速度
    O33 = diag([ 0 0 0 ]);
    O31 = [ 0 0 0]';
%% Q:系统噪声方差阵 更新
    q11 = (1-exp(-2*alpha*T)+2*alpha*T+(2*alpha^3*T^3)/3-2*alpha^2*T^2-4*alpha*T*exp(-alpha*T))/(2*alpha^4);
    q12 = (exp(-2*alpha*T)+1-2*exp(-alpha*T)+2*alpha*T*exp(-alpha*T)-2*alpha*T+alpha^2*T^2)/(2*alpha^3);
    q13 = (1-exp(-2*alpha*T)-2*alpha*T*exp(-alpha*T))/(2*alpha^2);
    q22 = (4*exp(-alpha*T)-3-exp(-2*alpha*T)+2*alpha*T)/(2*alpha^2);
    q23 = (exp(-2*alpha*T)+1-2*exp(-alpha*T))/(2*alpha);
    q33 = (1-exp(-2*alpha*T))/(2); 
      
    Q_tmp = [ q11 q12 q13   %  ?? 是否整体 subQ = a*subQ;????
             q12 q22 q23
             q13 q23 q33 ];

    if (acc_cur >= 0 ) % 根据当前加速度的正负??Xk1??  调整加速度方差
        R = (4-pi)*(a_max(2) - acc_cur)^2/pi; % ax_max=[-ax_max, ax_max]X方向的最大加速度
    else
        R = (4-pi)*(a_max(1) + acc_cur)^2/pi;
    end
    
    Q = 2*R*Q_tmp;
%% 状态更新
    
    Fai = [ 1  T  (-1+alpha*T+exp(-alpha*T))/(alpha^2);  % 一个维度
               0  1  (1-exp(-alpha*T))/alpha;
               0  0  exp(-alpha*T) ];      

    U = [ (-T + alpha*T^2 + ( 1-exp(-alpha*T) )/alpha )/ alpha;  % 一个维度
              T - ( 1-exp(-alpha*T) )/alpha;
               1 - exp(-alpha*T) ]; 
     
    Xk_new = Fai*Xk + U*E_a;  
end