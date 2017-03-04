function [x]=motion(x, u)
    
%     d_t = (u(1) + u(2))/2;
%     d_r = (u(1) - u(2))/(2*B);
%     
%     x = x + F*[d_t*cos(x(3) + d_r) d_t*sin(x(3) + d_r) 2*d_r ];
    
    global dt;
    global PoseSize;
    global LMSize;
    
    F = horzcat(eye(PoseSize),zeros(PoseSize,LMSize*GetnLM(x)));
    B = [dt*cos(x(3)) 0;
         dt*sin(x(3)) 0;
         0 dt];
     
    x = x+F'*B*u;  %  (3 x 3)'*(3 x 2)*(2x1)         (3 * 5)' * (3 * 2) * (2 * 1) ==> 5*1
    x(3) = PI2PI(x(3));

end