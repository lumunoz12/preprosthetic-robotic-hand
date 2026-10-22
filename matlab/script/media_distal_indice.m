clear
clc
close all

max_iteraciones=1000;

theta_rot = 0*pi/180;

rot = @(x,y) deal( ...
    x*cos(theta_rot) - y*sin(theta_rot), ...
    x*sin(theta_rot) + y*cos(theta_rot) );

% ===== MATRIZ PARA GUARDAR ÁNGULOS =====
Angles = zeros(361, 4);
%th1_new = zeros(361,1);
%th2_new = zeros(361,1);
th1_new_indice = zeros(361,1);
th2_new_indice = zeros(361,1);


for i=0:360

    AB = 38.16;
    BC = 7.56;
    CD = 38;
    AD = 8.64;

    th0=0; 
    th1=i*pi/180; 
    th2est=54.57*pi/180; 
    th3est=144.57*pi/180;

    F=[1;1];
    iteracion_actual=0;

    while norm(F)>=1.0e-10 && iteracion_actual<max_iteraciones
        
        f1=AB*cos(th1)+BC*cos(th1+th2est)+CD*cos(th1+th2est+th3est)-AD*cos(th0);
        f2=AB*sin(th1)+BC*sin(th1+th2est)+CD*sin(th1+th2est+th3est)-AD*sin(th0);

        A=[(-BC*sin(th1+th2est)-CD*sin(th1+th2est+th3est)) (-CD*sin(th1+th2est+th3est));...
           (BC*cos(th1+th2est)+CD*cos(th1+th2est+th3est)) (CD*cos(th1+th2est+th3est))];

        B=[(-f1);(-f2)];

        X=A\B;

        th2est=th2est + X(1);
        th3est=th3est + X(2);

        f1=AB*cos(th1)+BC*cos(th1+th2est)+CD*cos(th1+th2est+th3est)-AD*cos(th0);
        f2=AB*sin(th1)+BC*sin(th1+th2est)+CD*sin(th1+th2est+th3est)-AD*sin(th0);

        F=[f1;f2];

        iteracion_actual=iteracion_actual+1;
    end

    if iteracion_actual>=max_iteraciones
        disp(['No se pudo converger para el ángulo ', num2str(i)]);
        continue;
    end

    % ===== GUARDAR ÁNGULOS =====
    Angles(i+1, :) = [(th1*180/pi)-40, (th2est*180/pi)-51.758, th3est*180/pi, (th1+th2est+th3est)*180/pi];

    %% Puntos
    Bx=AB*cos(th1); By=AB*sin(th1);
    Cx=AB*cos(th1)+BC*cos(th1+th2est);
    Cy=AB*sin(th1)+BC*sin(th1+th2est);
    Dx=AB*cos(th1)+BC*cos(th1+th2est)+CD*cos(th1+th2est+th3est);
    Dy=AB*sin(th1)+BC*sin(th1+th2est)+CD*sin(th1+th2est+th3est);

    [Bxr, Byr] = rot(Bx,By);
    [Cxr, Cyr] = rot(Cx,Cy);
    [Dxr, Dyr] = rot(Dx,Dy);

    %th1_new(i+1)= (atan2(Byr, Bxr))*(180/pi)-40;
    %th2_new(i+1)= ((atan2(Cyr-Byr, Cxr-Bxr))*(180/pi))-54.58;

    th1_new_indice(i+1)= Angles(i+1,1);
    th2_new_indice(i+1) = Angles(i+1,2);
    plot(Bxr,Byr,'bh'); hold on
    plot(Cxr,Cyr,'gh');
    plot(Dxr,Dyr,'bh');
    plot(0,0,'kh');

    grid on
    axis equal
end

%% =============================
% LIMPIAR DATOS
%% =============================

% Eliminar NaN
valid = ~isnan(th1_new_indice) & ~isnan(th2_new_indice);
th1_new_indice = th1_new_indice(valid);
th2_new_indice = th2_new_indice(valid);

% Ordenar por th1
[th1_sorted, idx] = sort(th1_new_indice);
th3_sorted = th2_new_indice(idx);

% Eliminar duplicados
[th1_unique2_indice, idx2] = unique(th1_sorted);
th3_unique2_indice = th3_sorted(idx2);

flexion_indice = [th1_unique2_indice th3_unique2_indice];

save flexion_indice th1_unique2_indice th3_unique2_indice

load flexion_indice   

PIP_indice = input('Ingrese th1_new en grados: ');

if PIP_indice < min(th1_unique2_indice) || PIP_indice > max(th1_unique2_indice)
    error('th1 fuera del rango válido del mecanismo')
end

DIP_indice = interp1(th1_unique2_indice, th3_unique2_indice, PIP_indice, 'pchip');

%% =============================
% RESULTADO
%% =============================

fprintf('Para th1 = %.3f° → th2 = %.3f°\n', PIP_indice, DIP_indice);