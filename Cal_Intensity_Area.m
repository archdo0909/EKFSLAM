function [I]=Cal_Intensity_Area(Detector, L_max)

    I= 1000*(4*pi*L_max*sqrt(2))/(Detector * Detector * 10^(-4));


end