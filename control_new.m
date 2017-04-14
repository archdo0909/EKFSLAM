function u = control_new(time, time_mv)

    T = 10; %[sec]
    
    V = 1.0; %[m/s]
    yawrate = 5; %[deg/s]
    
    u = [V*(1-exp(-time/T)) toradian(yawrate)*(1-exp(-time_mv/T))]';
    
end