function [Y_accN] = DataFromAccelerometerY(x_pos,y_pos,t,STD)

%claculate Velocity 
d_t = diff(t);
d_y = diff(y_pos);
Y_velocity = d_y ./ d_t;

% Calculate accelerate 
dd_y = diff(Y_velocity);
Y_acc = dd_y ./ d_t(1:end-1);

% Add Noise
Noise = STD * randn(size(Y_acc));  % zero-mean, std = STD
Y_accN = Y_acc + Noise;


end