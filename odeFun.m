function dx = odeFun(x,A,B)
dx = zeros(7,1);
k = 1;
dx(1:5) = A*x(1:5) + B*x(6:7);
dx(6:7) = - k * x(6:7);
%dx(6:7) = -k * x(6:7);
end

