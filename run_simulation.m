clear;clc;close all;

%define parameters:
params.m=1.2;
params.g=9.81;
params.Ixx=0.01;
params.Iyy=0.01;
params.Izz=0.02;
params.l = 0.25;  % Arm length

t_span = [0 5];

%Initial Conditions    X=[x y z vx vy vz phi theta psi p q r]
X0 = [0;0;0;0;0;0;0;0;0;0;0;0];
U = [params.m * params.g;0;0;0];

% we use ode45 to solve the diffrenrial eaquations
[t,X_out] = ode45(@(t,X) quad_dynamics(t,X,U,params),t_span,X0);

%plot:

figure();
subplot(3,1,1);
plot(t,X_out(:,1));
title('x position');
ylabel('Position(m)');
grid on;

subplot(3,1,2);
plot(t,X_out(:,2));
title('y position');
ylabel('Position(m)');
grid on;

subplot(3,1,3);
plot(t,X_out(:,3));
title('z position');
ylabel('Position(m)');
grid on;
xlabel('Time(s)');