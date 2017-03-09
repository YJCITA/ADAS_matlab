%%
% 比较三种装换方法：
% 1.统一转到导航系下，再算k-1到k时刻的余弦矩阵
% 2.直接用欧拉角变化量，计算（这个方法是错的，因为欧拉角做差是没有意义的）
% 3.用姿态变化四元数进行计算（采用这种方法，初步验证OK）
%
% 注意：姿态变化四元数q_h的计算，使用的应该是陀螺仪的dt时间内的角增量
% 此处采用欧拉角变化量，因为：k-1时刻的欧拉角为0（所以F为阵为单位阵），所以小角度变化时，可以近似认为欧拉角变化量跟陀螺仪角增量相等
% 所以，计算结果上也可以看出，小欧拉角变化时三种方法计算出来的阵相同，角度大的时候就会有差异

%%
clear;
clc
% x y z
syms fai theta psi  dfai dtheta dpsi 
% fai = 0/pi;
% theta = 0/pi;
% psi = 0/pi;
% 
% dfai = 0.01/pi;
% dtheta = 0.01/pi;
% dpsi = 0.001/pi;


% k-1
C3_k_1 = [1  0  0;
          0 cos(fai) sin(fai);
          0 -sin(fai) cos(fai)];

C2_k_1 = [cos(theta) 0 -sin(theta);
                0    1    0;
            sin(theta) 0  cos(theta)];

C1_k_1 = [cos(psi) sin(psi) 0;
        -sin(psi) cos(psi) 0;
           0       0      1];
% k
 C3_k = [1  0  0;
        0 cos(fai+dfai) sin(fai+dfai);
        0 -sin(fai+dfai) cos(fai+dfai)];

C2_k = [cos(theta+dtheta) 0 -sin(theta+dtheta);
            0    1    0;
        sin(theta+dtheta) 0  cos(theta+dtheta)];

C1_k = [cos(psi+dpsi) sin(psi+dpsi) 0;
        -sin(psi+dpsi) cos(psi+dpsi) 0;
           0       0      1];


Ctk_n = C3_k*C2_k*C1_k;
Cn_tk_1 = (C3_k_1*C2_k_1*C1_k_1)';

% Ctk_n = expand(C3_k)*expand(C2_k)*expand(C1_k);
% Cn_tk_1 = (expand(C3_k_1)*expand(C2_k_1)*expand(C1_k_1))';
Ctk_tk_1 = Ctk_n*Cn_tk_1

% delta
C3_d = [1  0  0;
      0 cos(dfai) sin(dfai);
      0 -sin(dfai) cos(dfai)];

C2_d = [cos(dtheta) 0 -sin(dtheta);
                0    1    0;
            sin(dtheta) 0  cos(dtheta)];

C1_d = [cos(dpsi) sin(dpsi) 0;
        -sin(dpsi) cos(dpsi) 0;
           0       0      1];
Ctk_tk_1_d = C3_d*C2_d*C1_d
 

% 四元数
d_angle = sqrt(dfai^2 + dtheta^2 + dpsi^2);
sin_d_2 = sin(d_angle/2);
cos_d_2 = cos(d_angle/2);
q_h = [cos_d_2, dfai/d_angle*sin_d_2,  dtheta/d_angle*sin_d_2,  dpsi/d_angle*sin_d_2 ];
q0 = q_h(1);
q1 = q_h(2);
q2 = q_h(3);
q3 = q_h(4);

C_q = [q0^2+q1^2-q2^2-q3^2  2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2)
        2*(q1*q2+q0*q3) q0^2-q1^2+q2^2-q3^2 2*(q2*q3-q0*q1)
        2*(q1*q3-q0*q2)  2*(q2*q3+q0*q1)   q0^2-q1^2-q2^2+q3^2]'




% Ctk_tk_1_d = C3_d*C2_d*C1_d


