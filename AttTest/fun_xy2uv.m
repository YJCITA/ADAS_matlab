% 投影变化
function [ uv ] = fun_xy2uv( M1, Ratt_new,Pc, xy )
    x = xy(1);
    y = xy(2);   
   
    Point_xyz = [x, y, 0, 1]';
    tt =Ratt_new*[I3 -Pc]*Point_xyz; 
    Z_c = tt(3);
    uv_new = M1*tt/Z_c;

    u = round(uv_new(1));
    v = round(uv_new(2));
    
    uv = [u v]';

end
