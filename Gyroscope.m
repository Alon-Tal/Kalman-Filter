function [w_degN] = Gyroscope(x_pos,y_pos,t,STD)

%claculate dx dy dt
d_t = diff(t);
d_x = diff(x_pos);
d_y = diff(y_pos);

theta_rad1 = atan2(d_y,d_x);
Cln_theta_rad_unwrapped = unwrap(theta_rad1);
theta_deg = rad2deg(Cln_theta_rad_unwrapped);

% calculate w
d_theta_deg = diff (theta_deg);
w_deg = d_theta_deg ./ d_t(2:end);

% Add Noise
Noise = STD * randn(size(w_deg));  % zero-mean, std = STD
w_degN = w_deg + Noise;

end