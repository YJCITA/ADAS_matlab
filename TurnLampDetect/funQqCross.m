% ËÄÔªÊý²æ³Ë
function [ Q ] = funQqCross(Q,q_h )
    q0 = q_h(1);
    q1 = q_h(2);
    q2 = q_h(3);
    q3 = q_h(4);
    Mq = [q0  -q1  -q2  -q3;
          q1   q0   q3  -q2;
          q2  -q3   q0   q1;
          q3   q2  -q1   q0];
      
     Q = Mq*Q;
end

