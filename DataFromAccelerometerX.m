function [X_accN] = DataFromAccelerometerX(x_pos,y_pos,t,STD)

%claculate Velocity 
d_t = diff(t);
d_x = diff(x_pos);
X_velocity = d_x ./ d_t;

% Calculate accelerate 
dd_x = diff(X_velocity);
X_acc = dd_x ./ d_t(1:end-1);

% Add Noise
Noise = STD * randn(size(X_acc));  % zero-mean, std = STD
X_accN = X_acc + Noise;

end