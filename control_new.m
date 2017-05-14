function u = control_new(time)

    %T = 10; %[sec]
%     if (time >= 24.3)
%        time = time-24.1; 
%     end
      V = 1.0; %[m/s]
      length = V * time;
      if (length >= 24)
         length = length-24; 
      end
%      %yawrate [rad/s]
%      if (time >= 6 && time <= 6.1)
%          yawrate = 900;
%          u = [V toradian(yawrate)]';
%      elseif (time >= 12 && time <= 12.1)
%          yawrate = 900;
%          u = [V toradian(yawrate)]';
%      elseif (time >= 18 && time <= 18.1)
%          yawrate = 900;
%          u = [V toradian(yawrate)]';
%      elseif (time >= 24.1 && time <= 24.2)
%          yawrate = 900;
%          u = [V toradian(yawrate)]';
%      else
%          yawrate = 0;
%          u = [V toradian(yawrate)]';
%      end     
     if (length >= 6 && length <= 6.1)
         yawrate = 900;
         u = [V toradian(yawrate)]';
     elseif (length >= 12 && length <= 12.1)
         yawrate = 900;
         u = [V toradian(yawrate)]';
     elseif (length >= 18 && length <= 18.1)
         yawrate = 900;
         u = [V toradian(yawrate)]';
     elseif (length >= 24 && length <= 24.1)
         yawrate = 900;
         u = [V toradian(yawrate)]';
     else
         yawrate = 0;
         u = [V toradian(yawrate)]';
     end     

    
     
end