function [ fh ] = fun_fhan( x1, x2, r, h0 )
   d = r*h0*h0;
   a0 = h0*x2;
   y = x1+a0;
   a1 = sqrt( d*(d + abs(y)*8.0) );
   a2 = a0 + sign(y)*(a1-d)*0.5;
   sy = (sign(y + d) - sign(y - d))*0.5;
   a = (a0  +y - a2)*sy + a2;
   sa = (sign(a + d) - sign(a - d))*0.5;

   fh = (-r*(a/d - sign(a))*sa - r*sign(a));
   
end

