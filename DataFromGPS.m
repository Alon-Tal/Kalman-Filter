function [LatitudeN, LongitudeN, theta_degN] = DataFromGPS(x_pos,y_pos,t,GPS_std,Theta_std)

%claculate dx dy dt
d_t = diff(t);
d_x = diff(x_pos);
d_y = diff(y_pos);

theta_rad1 = atan2(d_y,d_x);
Cln_theta_rad_unwrapped = unwrap(theta_rad1);
theta_deg = rad2deg(Cln_theta_rad_unwrapped);

% Add Noise
Noise1 = GPS_std * randn(size(x_pos));  % zero-mean, std = GPS_std
LatitudeN = x_pos + Noise1;
Noise2 = GPS_std * randn(size(y_pos));  % zero-mean, std = GPS_std
LongitudeN = y_pos + Noise2;
Noise3 = Theta_std * randn(size(theta_deg));  % zero-mean, std = Theta_std
theta_degN = theta_deg + Noise3;

%reduce number of samples
LatitudeN = LatitudeN(10:10:end);
LongitudeN = LongitudeN(10:10:end);
theta_degN = theta_degN(10:10:end);

end