function y = ekfMeasurement(xk)
    y = zeros(7,1);
    y(1:6) = xk(1:6);
    y(7)   = xk(9);
end