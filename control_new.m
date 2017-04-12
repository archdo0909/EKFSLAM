function u = control_new(time, time_ob)

    T = 10; %[sec]
    
    V = 1.0; %[m/s]
    yawrate = 5; %[deg/s]
    
    u = [V*(1-exp(-time_ob/T)) toradian(yawrate)*(1-exp(-time/T))]';
    
end