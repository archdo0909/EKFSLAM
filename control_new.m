function u = control_new(time)

    %T = 10; %[sec]
    
    V = 1.0; %[m/s]
%     if (time >= 3 && time < 6) 
%         yawrate = 90; %[deg/s]
%         u = [V toradian(yawrate)]';
%     elseif (time >= 6 && time <= 9)  
%         yawrate = 5;
%         u = [V toradian(yawrate)]';
%     else
%         yawrate = 0;
%         u = [V toradian(yawrate)]';
%     end

    if (time == 3)
        yawrate = 900;
        u = [V toradian(yawrate)]';
    else
        yawrate = 0;
        u = [V toradian(yawrate)]';
    end
    %u = [V*(1-exp(-time/T)) toradian(yawrate)*(1-exp(-time_mv/T))]';

end