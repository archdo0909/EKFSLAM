close all;
clear all;
clc

time = 0;
endtime = 9;
global dt;
dt = 0.1;
nSteps = ceil((endtime - time)/dt);
t_max = 2;

result.u = [];
for i = 1:nSteps
    
      if time_ob > 3 
        time_ob = 0;
     end 
    
    
     time = time + dt;
     time_ob = time_ob + dt;
     u=control_new(time, time_ob); 
    
    
    result.u = [result.u; u' time];
end
csvwrite('u.csv', result.u)