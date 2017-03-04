function [G]=JacobianG(X, u, B)

    d_r = (u(1)-u(2))/(2*B);
    d_t = (u(1)+u(2))/2;
    
    G = [(1/2)*cos(d_r + X(3)) - (d_t/(2*B))*sin(d_t + X(3)) (1/2)*cos(d_r + X(3)) + (d_t/(2*B))*sin(d_t + X(3));
        (1/2)*sin(d_r + X(3)) + (d_t/(2*B))*cos(d_t + X(3)) (1/2)*sin(d_r + X(3)) - (d_t/(2*B))*cos(d_t + X(3))
        1 / B -1/B];
    
end
    