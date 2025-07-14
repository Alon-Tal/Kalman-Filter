close all; clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creating a path
% 1. path is rise sin & cos
% 2. Path is sin & cos
% 3. path is X=Linear y=sin
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

choice = input('Choose an option: \n 1 - rise sin & cos \n 2 - sin & cos \n 3 - X=Linear y=sin 2 \n your choise: ');
tstart = 0; % the time to start drive
tstop = 10.01; %how much time the car drive
txlimit=floor(tstop);
%xlim([tstart txlimit])
dt = 0.01; % time sample of 0.01
t = tstart:dt:tstop; % create the time vector

switch choice
    case 1
        Cln_x_pos = t/pi.*cos(t);
        Cln_y_pos = t/pi.*sin(t);

    case 2
        mult_factor = 1;
        Cln_x_pos = mult_factor * cos(t);
        Cln_y_pos = mult_factor * sin(t);
      
    case 3
        mult_factor = 1;
        Cln_x_pos = linspace(tstart,tstop,length(t));
        Cln_y_pos = mult_factor * sin(t);

    otherwise
        disp('Invalid selection');
end

%%%%% this section is only for calculate thete to plot it without noise
d_x = diff(Cln_x_pos);
d_y = diff(Cln_y_pos);

theta_rad1 = atan2(d_y,d_x);
Cln_theta_rad_unwrapped = unwrap(theta_rad1);
Cln_thetaDeg = rad2deg(Cln_theta_rad_unwrapped);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the Clean Path (without Noise) - X,Y,Theta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('name','Path Without Noise')
sgtitle('Path Without Noise');
subplot(3,1,1) % for X
plot(t,Cln_x_pos)
ylabel('X path [m]')
xlim([tstart txlimit])
subplot(3,1,2) % for Y
plot(t,Cln_y_pos)
ylabel('Y path [m]')
xlim([tstart txlimit])
subplot(3,1,3) % for theta
plot(t(1:end-1),Cln_thetaDeg)
ylabel('theta [deg]')
xlabel('Time [s]')
xlim([tstart txlimit])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get Data from Sensors (With Adding Noise)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

           %%%%% the STandard Deviation %%%%%
Std_AccelX = 0.3;           % Accelerometer X std [m/s^2]
Std_AccelY = 0.3;           % Accelerometer Y std [m/s^2]
Std_Gyro  = 0.3;             % Gyroscope std [deg/s]
Std_GPS   = 3;            % GPS std [m]
Std_Theta = 3;             % GPS angle std [deg]


[SenAcc_x] = DataFromAccelerometerX(Cln_x_pos,Cln_y_pos,t,Std_AccelX);
[SenAcc_y] = DataFromAccelerometerY(Cln_x_pos,Cln_y_pos,t,Std_AccelY);
[SenGyro_w_deg] = Gyroscope(Cln_x_pos,Cln_y_pos,t,Std_Gyro);
[GPS_Latitude, GPS_Longitude, GPS_thetaDeg] = DataFromGPS(Cln_x_pos,Cln_y_pos,t,Std_GPS,Std_Theta);
GPS_t = t(10:10:end);
t = t(1:end-2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the Path for GPS (with Noise) - X,Y,Theta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('name','GPS Path With Noise')
sgtitle('GPS Path With Noise');
subplot(3,1,1) % for Latitude ~ X
plot(GPS_t,GPS_Latitude)
ylabel('GPS Latitude [m]')
subplot(3,1,2) % for Longitude ~ Y
plot(GPS_t,GPS_Longitude)
ylabel('GPS Longitude [m]')
subplot(3,1,3) % for theta
plot(GPS_t,GPS_thetaDeg)
ylabel('theta [deg]')
xlabel('Time [s]')


figure('name','Acceleration and W')
sgtitle('Acceleration and W');
subplot(3,1,1) % for accx
plot(t,SenAcc_x)
ylabel('Acc X $$\frac{m^2}{s}$$' ,'Interpreter', 'latex')
xlim([tstart txlimit])
subplot(3,1,2) % for accy
plot(t,SenAcc_y)
ylabel('Acc Y $$\frac{m^2}{s}$$' ,'Interpreter', 'latex')
xlim([tstart txlimit])
subplot(3,1,3) % for w
plot(t,SenGyro_w_deg)
ylabel('$$\omega\ [^\circ\!/\!s]$$', 'Interpreter', 'latex');
xlabel('Time [s]' ,'Interpreter', 'latex')
xlim([tstart txlimit])


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Matrix for Kalman filter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Xk_0 = [Cln_x_pos(1);      %X     
        Cln_y_pos(1);      %Y     
        (Cln_x_pos(2) - Cln_x_pos(1)) / dt;    %Vx  
        (Cln_y_pos(2) - Cln_y_pos(1)) / dt;    %Vy
        0;       %ax
        0;       %ay
        Cln_thetaDeg(1);   %theta Deg
        SenGyro_w_deg(1);  %w deg
];
Xk(:,1) = Xk_0;

p_0 = 0.1 * eye(8);
% p_0 = [ 1 0 0 0 0 0 0 0;
%         0 1 0 0 0 0 0 0;
%         0 0 1 0 0 0 0 0;
%         0 0 0 1 0 0 0 0;
%         0 0 0 0 1 0 0 0;
%         0 0 0 0 0 1 0 0;
%         0 0 0 0 0 0 1 0;
%         0 0 0 0 0 0 0 1;
% ];
Pk(:,:,1) = p_0;

F = [   1 0 dt 0  (dt^2)/2 0        0 0;  %x
        0 1 0  dt 0        (dt^2)/2 0 0;  %y
        0 0 1  0  dt       0        0 0;  %vx
        0 0 0  1  0        dt       0 0;  %vy
        0 0 0  0  0        0        0 0;  %ax
        0 0 0  0  0        0        0 0;  %ay
        0 0 0  0  0        0        1 dt; %theta deg
        0 0 0  0  0        0        0 0;  %w
];

B = [   0 0 0 0 0 0 0 0; %x
        0 0 0 0 0 0 0 0; %y
        0 0 0 0 0 0 0 0; %vx
        0 0 0 0 0 0 0 0; %vy
        0 0 0 0 1 0 0 0; %ax
        0 0 0 0 0 1 0 0; %ay
        0 0 0 0 0 0 0 0; %theta deg
        0 0 0 0 0 0 0 1; %w
];

% Std_AccelX^2   Std_AccelY^2   Std_Gyro^2   Std_GPS^2   Std_Theta^2
Qk = [  0 0 0 0 0             0             0 0;          %x
        0 0 0 0 0             0             0 0;          %y
        0 0 0 0 0             0             0 0;          %vx
        0 0 0 0 0             0             0 0;          %vy
        0 0 0 0 Std_AccelX^2  0             0 0;          %ax
        0 0 0 0 0             Std_AccelY^2  0 0;          %ay
        0 0 0 0 0             0             0 0;          %theta
        0 0 0 0 0             0             0 Std_Gyro^2; %w
];

R = [   Std_GPS^2   0           0;
        0           Std_GPS^2   0;
        0           0           Std_Theta^2;
];

H = [   1 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0;
        0 0 0 0 0 0 1 0;
];

I = eye(8);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Kalman filter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=2:length(SenAcc_x) % i=1 is initial conditions

    %%%%%%%%%%%%%%%%%%% Time Update ("Predict") %%%%%%%%%%%%%%%%%%%%%
    
    %1
    uk = [0; 0; 0; 0; SenAcc_x(i); SenAcc_y(i); 0; SenGyro_w_deg(i)];
    Xk(:,i) = F * Xk(:,i-1) + B * uk;
    %2
    Pk(:,:,i) = F * Pk(:,:,i-1) * F' + Qk;

    %%%%%%%%%%%%%%%%%%%% Measurement Update ("Correct") %%%%%%%%%%%%%
    if mod(i,10) == 0 

        %1
        Kk = Pk(:,:,i) * H' * inv(H * Pk(:,:,i) * H' + R) ;
        %2
        zk = [GPS_Latitude(i/10); GPS_Longitude(i/10); GPS_thetaDeg(i/10)];
        Xk(:,i) = Xk(:,i) + Kk * (zk - H * Xk(:,i));
        %3
        Pk(:,:,i) = (I - Kk * H) * Pk(:,:,i);

    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the Path After Kalman Filter - X,Y,Theta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('name','Path After Kalman Filter')
sgtitle('Path After Kalman Filter');
subplot(3,1,1) % for Latitude ~ X
plot(t,Xk(1, :))
ylabel('X [m]')
subplot(3,1,2) % for Longitude ~ Y
plot(t,Xk(2, :))
ylabel('Y [m]')
subplot(3,1,3) % for theta
plot(t,Xk(7, :))
ylabel('theta [deg]')
xlabel('Time [s]')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the Path After Kalman Filter with GPS with Real path - X,Y,Theta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('name','Kalman & GPS & Real')
sgtitle('Path After Kalman Filter');

subplot(3,1,1) % for Latitude ~ X
plot(t,Xk(1, :),'r', 'LineWidth', 1)
hold on
plot(GPS_t+9*dt, GPS_Latitude, 'bo', 'LineWidth', 1);
plot(t,Cln_x_pos(1:end-2), 'k', 'LineWidth', 1)
ylabel('X [m]')
legend('After Kalman', 'GPS Data', 'Real (NoNoise)')
xlim([tstart txlimit])

subplot(3,1,2) % for Longitude ~ Y
plot(t,Xk(2, :),'r', 'LineWidth', 1)
hold on
plot(GPS_t+9*dt, GPS_Longitude, 'bo', 'LineWidth', 1);
plot(t,Cln_y_pos(1:end-2), 'k', 'LineWidth', 1)
ylabel('Y [m]')
legend('After Kalman', 'GPS Data', 'Real (NoNoise)')
xlim([tstart txlimit])

subplot(3,1,3) % for theta
plot(t,Xk(7, :),'r', 'LineWidth', 1)
hold on
plot(GPS_t+9*dt, GPS_thetaDeg, 'bo', 'LineWidth', 1);
plot(t,Cln_thetaDeg(1:end-1), 'k', 'LineWidth', 1)
ylabel('theta [deg]')
legend('After Kalman', 'GPS Data', 'Real (NoNoise)')
xlim([tstart txlimit])

xlabel('Time [s]')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot of the difference - X,Y,Theta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('name','The Difference')
sgtitle('the difference');

subplot(3,1,1) % for Latitude ~ X

plot(t, Xk(1, :) - Cln_x_pos(1:end-2), 'r');
hold on;
plot(GPS_t, GPS_Latitude - Cln_x_pos(10:10:end), 'b');
ylabel('delta X [m]')
legend('Kalman - Real', 'GPS Data - Real')

subplot(3,1,2) % for Longitude ~ Y
plot(t, Xk(2, :) - Cln_y_pos(1:end-2), 'r');
hold on;
plot(GPS_t, GPS_Longitude - Cln_y_pos(10:10:end), 'b');
ylabel('delta y [m]')
legend('Kalman - Real', 'GPS Data - Real')

subplot(3,1,3) % for theta
plot(t, Xk(7, :) - Cln_thetaDeg(1:end-1), 'r');
hold on;
plot(GPS_t, GPS_thetaDeg - Cln_thetaDeg(10:10:end), 'b');
ylabel('delta theta [deg]')
legend('Kalman - Real', 'GPS Data - Real')

xlabel('Time [s]')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Error & Sigma - X,Y,Theta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('name','The Error')
sgtitle('the Error');

subplot(3,1,1) % for Latitude ~ X

plot(t, abs(Xk(1, :) - Cln_x_pos(1:end-2)), 'r');
hold on;
a=squeeze(Pk(1,1,:))';
plot(t,3*sqrt(a), 'b')
ylabel('delta X  [m]')
legend('Kalman - Real', 'Sig x')

subplot(3,1,2) % for Longitude ~ Y
plot(t, abs(Xk(2, :) - Cln_y_pos(1:end-2)), 'r');
hold on;
a=squeeze(Pk(2,2,:))';
plot(t,3*sqrt(a), 'b')
ylabel('delta y  [m]')
legend('Kalman - Real', 'Sig y')

subplot(3,1,3) % for theta
plot(t, abs(Xk(7, :) - Cln_thetaDeg(1:end-1)), 'r');
hold on;
a=squeeze(Pk(7,7,:))';
plot(t,3*sqrt(a), 'b')
ylabel('delta theta [deg]')
legend('Kalman - Real', 'Sig theta')

xlabel('Time [s]');