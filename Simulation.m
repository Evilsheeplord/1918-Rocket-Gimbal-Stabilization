% Rocket_Trajectory_Simulation.m
% Multi-stage rocket dynamics and trajectory simulation.
%
% Description:
% Predicts multi-stage rocket dynamics and trajectories based on the given 
% rocket mass, engine thrust, launch parameters, and drag coefficient.
%
% Variable List:
% Delta = Time step (s)
% t = Time (s)
% Thrust = Thrust (N)
% Mass = Mass (kg)
% Mass_Rocket_With_Motor = Mass with motor (kg)
% Mass_Rocket_Without_Motor = Mass without motor (kg)
% beta = Angle (deg)
% C = Drag coefficient
% Rho = Air density (kg/m^3)
% A = Rocket projected area (m^2)
% Gravity = Gravity (m/s^2)
% Launch_Rod_Length = Length of launch rod (m)
% n = Counter
% Fn = Normal force (N)
% Drag = Drag force (N)
% Fx = Sum of forces in the horizontal direction (N)
% Fy = Sum of forces in the vertical direction (N)
% Vx = Velocity in the horizontal direction (m/s)
% Vy = Velocity in the vertical direction (m/s)
% Ax = Acceleration in the horizontal direction (m/s^2)
% Ay = Acceleration in the vertical direction (m/s^2)
% x = Horizontal position (m)
% y = Vertical position (m)
% Distance_x = Horizontal distance travelled (m)
% Distance_y = Vertical travelled (m)
% Distance = Total distance travelled (m)
% Memory_Allocation = Maximum number of time steps expected

clear, clc      % Clear command window and workspace

% Parameters
Delta = 0.001;                  % Time step 
Memory_Allocation = 30000;      % Maximum number of time steps expected

% Preallocate memory for arrays
t = zeros(1, Memory_Allocation);
Thrust = zeros(1, Memory_Allocation);
beta = zeros(1, Memory_Allocation);
gimbal_angle = zeros(1, Memory_Allocation);
Fn = zeros(1, Memory_Allocation);
Drag = zeros(1, Memory_Allocation);
Fx = zeros(1, Memory_Allocation);
Fy = zeros(1, Memory_Allocation);
Ax = zeros(1, Memory_Allocation);
Ay = zeros(1, Memory_Allocation);
Vx = zeros(1, Memory_Allocation);
Vy = zeros(1, Memory_Allocation);
x = zeros(1, Memory_Allocation);
y = zeros(1, Memory_Allocation);
Distance_x = zeros(1, Memory_Allocation);
Distance_y = zeros(1, Memory_Allocation);
Distance = zeros(1, Memory_Allocation);
w_dot = zeros(1, Memory_Allocation);
w = zeros(1, Memory_Allocation);

delay = 100;
C_d = 0.4;                                % Drag coefficient
Rho = 1.2;                              % Air density (kg/m^3)
A = 4.9*10^-4;                          % Rocket projected area (m^2)
Gravity = 9.81;                         % Gravity (m/s^2)
Launch_Rod_Length = 1;                  % Length of launch rod (m)
Mass = 0.35;          % Initial rocket mass (kg)
Moment_Of_Inertia = 5*(1/4*Mass*(4.026/100)^2 + 1/12*Mass*(0.535)^2);
Response_Time = 0;
P = 0.1;
I = 0;
D = 0.135;

beta(1) = 5/180*pi;            % Initial angle (deg)
gimbal_angle(1) = 0;
Vx(1) = 0;                      % Initial horizontal speed (m/s)
Vy(1) = 0;                      % Initial vertical speed (m/s)
x(1) = 0;                       % Initial horizontal position (m)
y(1) = 0.1;                     % Initial vertical position (m)
Distance_x(1) = 0;              % Initial horizontal distance travelled (m)
Distance_y(1) = 0;              % Initial vertical distance travelled (m)
Distance(1) = 0;                % Initial  distance travelled (m)


for n = 2:Memory_Allocation                  % Run until rocket hits the ground
 
    t(n)= (n-1)*Delta;          % Elapsed time                     

    % Determine rocket thrust and mass based on launch phase
    if t(n) <= 0.2                              % Launch phase 1
        Thrust(n) = 70*t(n);  
    elseif t(n) > 0.2 && t(n) < 30             % Launch phase 2 Usually 1.9
        Thrust(n) = 4.5;                          
    else
        Thrust(n) = 0;
    end

    % Normal force calculations  
    if Distance(n-1) <= Launch_Rod_Length       % Launch rod normal force
        Fn(n) = Mass*Gravity*cos(beta(1));
    else
        Fn(n) = 0;                              % No longer on launch rod
    end
        
    % Lift and drag force calculation
%    Lift(n) = 0.5*C_l*Rho*(Vx(n-1)^2+Vy(n-1)^2)*A_lift;  %Calculate lift
    Drag(n) = 0.5*C_d*Rho*(Vx(n-1)^2+Vy(n-1)^2)*A;  %Calculate drag force
    
    % Sum of forces calculations 
    Fx(n)= Thrust(n)*-sin(beta(n-1)+gimbal_angle(n-1))+Drag(n)*sin(beta(n-1))...
        +Fn(n)*sin(beta(n-1));                            % Sum x forces
    Fy(n)= Thrust(n)*cos(beta(n-1)+gimbal_angle(n-1))-(Mass*Gravity)-...
        Drag(n)*cos(beta(n-1))+Fn(n)*cos(beta(n-1));    % Sum y forces
        
    % Acceleration calculations
    Ax(n)= Fx(n)/Mass;                       % Net accel in x direction 
    Ay(n)= Fy(n)/Mass;                       % Net accel in y direction
	
    % Velocity calculations
    Vx(n)= Vx(n-1)+Ax(n)*Delta;                 % Velocity in x direction
    Vy(n)= Vy(n-1)+Ay(n)*Delta;                 % Velocity in y direction
	
    % Position calculations
    x(n)= x(n-1)+Vx(n)*Delta;                   % Position in x direction
    y(n)= y(n-1)+Vy(n)*Delta;                   % Position in y direction
    
    % Distance calculations    
    Distance_x(n) = Distance_x(n-1)+abs(Vx(n)*Delta);      % Distance in x 
    Distance_y(n) = Distance_y(n-1)+abs(Vy(n)*Delta);      % Distance in y 
    Distance(n) = (Distance_x(n)^2+Distance_y(n)^2)^(1/2); % Total distance
    
    % Sum of moments calculation
    T = -Thrust(n)*sin(gimbal_angle(n-1));
    
    % Angular acceleration calculation
    w_dot(n) = T/Moment_Of_Inertia;
    w(n) = w(n-1)+w_dot(n)*Delta;
    
    beta(n) = beta(n-1)+w(n)*Delta;
    if Distance(n-1) <= Launch_Rod_Length       % Launch rod normal force
        gimbal_angle(n) = 0;
    else
        gimbal_angle(n) = P*beta(n-delay)+D*(beta(n-delay)-(beta(n-1-delay)))/Delta;  % No longer on launch rod
        if abs(gimbal_angle(n)) > 9/180*pi
            if gimbal_angle(n) > 0
                gimbal_angle(n) = 9/180*pi;
            else
                gimbal_angle(n) = -9/180*pi;
            end
        end
    end
end

figure('units','normalized','outerposition',[0 0 1 1]) % Maximize plot window

% Figure 2
subplot(2,2,1)
plot(t(1:n),Vx(1:n));
xlabel({'Time (s)'});
ylabel({'Vx (m/s)'});
title({'Horizontal Velocity'});

% Figure 3
subplot(2,2,2)
plot(t(1:n),Vy(1:n));
xlabel({'Time (s)'});
ylabel({'Vy (m/s)'});
title({'Vertical Velocity'});

% Figure 4
subplot(2,2,3)
plot(t(1:n),180/pi*beta(1:n));
xlabel({'Time (s)'});
ylabel({'Beta (Deg)'});
title({'Beta'});

% Figure 9
subplot(2,2,4)
plot(t(1:n),180/pi*gimbal_angle(1:n));
xlim([0 30]);
ylim([-10 10]);
xlabel({'Time (s)'});
ylabel({'Gimbal Angle (Deg)'});
title({'Gimbal Angle'});

%%
% Figure 7
subplot(3,3,7)
plot(t(1:n),Thrust(1:n));
xlim([0 3]);
xlabel({'Time (s)'});
ylabel({'Thrust (N)'});
title({'Thrust'});

% Figure 8
subplot(3,3,8)
plot(t(1:n),Drag(1:n));
xlabel({'Time (s)'});
ylabel({'Drag (N)'});
title({'Drag Force'});

% Figure 9
subplot(3,3,9)
plot(Distance(1:n),Fn(1:n));
xlim([0 2]);
xlabel({'Distance (m)'});
ylabel({'Normal Force (N)'});
title({'Normal Force'});

