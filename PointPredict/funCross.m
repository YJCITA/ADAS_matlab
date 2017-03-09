function [ C ] = funCross( A, B )
    C = zeros(3,1);
    C(1) = A(2)*B(3) - A(3)*B(2);
    C(2) = A(3)*B(1) - A(1)*B(3);
    C(3) = A(1)*B(1) - A(2)*B(1);

end

