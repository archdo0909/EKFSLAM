function u = control_new(time)

    %T = 10; %[sec]
    
     V = 1.0; %[m/s]
     %yawrate [rad/s]
     if (time >= 6 && time <= 6.1)
         yawrate = 900;
         u = [V toradian(yawrate)]';
     elseif (time >= 12 && time <= 12.1)
         yawrate = 900;
         u = [V toradian(yawrate)]';
     elseif (time >= 18 && time <= 18.1)
         yawrate = 900;
         u = [V toradian(yawrate)]';
     else
         yawrate = 0;
         u = [V toradian(yawrate)]';
     end     
%     %u = [V*(1-exp(-time/T)) toradian(yawrate)*(1-exp(-time_mv/T))]';

end