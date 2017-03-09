% 逆投影变化
function [ xy ] = fun_uv2xy( camera, uv )
    u = uv(1);
    v = uv(2);
    
    h = camera.h;
    gama0 = camera.gama0;
    theta0 = camera.theta0;
    alpha = camera.alpha;
    beta = camera.beta;
    m = camera.m;
    n = camera.n;
    
    
    t1 = h/tan(theta0 - alpha + v*2*alpha/(m-1));
    t2 = cos(gama0 - beta + u*2*beta/(n-1));
    t3 = sin(gama0 - beta + u*2*beta/(n-1));
    x = t1*t2;
    y = t1*t3;
    
    xy = [x y]';

end

