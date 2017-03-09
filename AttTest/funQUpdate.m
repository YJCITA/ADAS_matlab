% ËÄÔªÊý²æ³Ë
function [ Q_new ] = funQUpdate(Q,w, dt )
    q0 = Q(1);
    q1 = Q(2);
    q2 = Q(3);
    q3 = Q(4);
    
    gx = w(1);
    gy = w(2);
    gz = w(3);    
    
    dq0 = 0.5*(-q1 * gx - q2 * gy - q3 * gz);
	dq1 = 0.5*(q0 * gx + q2 * gz - q3 * gy);
	dq2 = 0.5*(q0 * gy - q1 * gz + q3 * gx);
	dq3 = 0.5*(q0 * gz + q1 * gy - q2 * gx); 

    q0 = q0 + dt*dq0;
	q1 = q1 + dt*dq1;
	q2 = q2 + dt*dq2;
	q3 = q3 + dt*dq3;
    
    Q_new = [q0, q1, q2, q3]';
end

