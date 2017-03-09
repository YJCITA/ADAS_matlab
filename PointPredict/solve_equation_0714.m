% syms x1 x2; %定义符号变量  
% syms y1 y2;  
% clc  
% [x1, x2] = solve('x1 + x2 = y1',...  
%     'x1^2 + x2^2 = y2',...  
%     'x1', 'x2') %求x1，x2用y1, y2表示的表达  

% syms u v u0 v0 a b p xw yw zw h z0 At real;
% A = [a 0 u0;
%      0 b v0;
%      0 0 1];
%  R = [1 0 0;
%      0 -cos(p) -sin(p);
%      0 sin(p)  -cos(p);];
%  t = [0 0 h/cos(p)]';
% eqns = z0*[u v 1]' - A*(R*[xw yw zw]' + t) == 0;
% [xw, yw, zw] = solve(eqns, 'xw', 'yw', 'zw')
 %求x1，x2用y1, y2表示的表达  



X1 = [5 4.4 5 4.5 3.6];
X2 = [5 5 5 4.5 3];
% syms x real;
% eqn = asin(X1(1)/(2*x)) + asin(X1(2)/(2*x)) + asin(X1(3)/(2*x)) + asin(X1(4)/(2*x)) + asin(X1(5)/(2*x)) - pi == 0;
% [x] = solve(eqn, 'x')

ymin = pi;
R_best = 0;
for gg = 3:0.01:5
    x = 0;
    for j = 1:5
        tt = X2(j)/(2*gg);
        x = x + asin(tt);
    end
    y = x - pi;
    if abs(y) < ymin
        ymin = abs(y);
        R_best = gg;
    end
        
end