% Control LQR de posicion usando:
%   Entrada  -> PWM
%   Salida   -> Velocidad angular
%

clear all
close all
clc

%% =========================================================
%  CARGAR DATOS
% ==========================================================

load("escalon_motor2.mat");

Ts = 20e-3;

u  = all_pwm(:); 
y1 = all_w(:);

N = length(u);

t = (0:N-1)' * Ts;
 

%% =========================================================
%  GRAFICOS DE DATOS
% ==========================================================

figure('Name','Datos Motor','NumberTitle','off')

subplot(2,1,1)
plot(t,u,'LineWidth',1.5)
grid on
xlabel('Tiempo [s]')
ylabel('PWM')
title('PWM')

subplot(2,1,2)
plot(t,y1,'LineWidth',1.5)
grid on
xlabel('Tiempo [s]')
ylabel('\omega [rad/s]')
title('Velocidad Angular')

%% =========================================================
%  IDENTIFICACION DEL SISTEMA
% ==========================================================

datos = iddata(y1,u,Ts);

% Modelo de primer orden
modelo = ssest(datos,1);

disp('Modelo identificado:')
modelo

A = modelo.A;
B = modelo.B;
C = modelo.C;
D = modelo.D;

%% =========================================================
%  MODELO AUMENTADO PARA POSICION
%
%  x1 = theta
%  x2 = omega
%
%  theta_dot = omega
% ==========================================================

%a = A(1,1);
%b = B(1,1);
a = modelo.A(1,1);

b = modelo.C(1,1)*modelo.B(1,1);

A_aug = [0 1;
         0 a];

B_aug = [0;
         b];

C_aug = [1 0];

D_aug = 0;

sys_aug = ss(A_aug,B_aug,C_aug,D_aug);

%% =========================================================
%  DISCRETIZAR SISTEMA
% ==========================================================

sysd = c2d(sys_aug,Ts,'zoh');

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

%% =========================================================
%  CONTROLADOR LQR
% ==========================================================

Q = diag([5000 50]);   % [posicion velocidad]
R = 0.001;

[K,P,E] = dlqr(Ad,Bd,Q,R);

disp('Ganancia LQR:')
disp(K)
%% =========================================================
%  SIMULACION
% ==========================================================

Tf = 4;                       % tiempo simulacion
Nsim = round(Tf/Ts);

x = zeros(2,Nsim);

% Referencia
theta_ref = pi/2;             % 90 grados

% Variables
u_control = zeros(1,Nsim);
pwm       = zeros(1,Nsim);
dir       = zeros(1,Nsim);

%% =========================================================
%  LOOP DE SIMULACION
% ==========================================================

for k = 1:Nsim-1

    % Error de estados
    x_error = x(:,k) - [theta_ref; 0];

    % Ley de control LQR
    control = -K * x_error;

    %% Saturacion PWM

    if control > 4095
        control = 4095;
    end

    if control < -4095
        control = -4095;
    end

    %% PWM y direccion

    pwm(k) = abs(control);

    if control >= 0
        dir(k) = -1;
    else
        dir(k) = 1;
    end

    u_control(k) = control;

    %% Sistema discreto

    x(:,k+1) = Ad*x(:,k) + Bd*control;

end

%% Evitar caida final del grafico

u_control(end) = u_control(end-1);
pwm(end)       = pwm(end-1);
dir(end)       = dir(end-1);

%% =========================================================
%  GRAFICOS DE SIMULACION
% ==========================================================

t_sim = (0:Nsim-1)*Ts;

figure('Name','Simulacion LQR','NumberTitle','off')

plot(t_sim,x(1,:)*180/pi,'LineWidth',1.5)
hold on
yline(theta_ref*180/pi,'r--')
grid on
ylabel('Posición [°]')
xlabel('Tiempo [s]')
legend('Posición actual', 'Objetivo', 'Location', 'best');
