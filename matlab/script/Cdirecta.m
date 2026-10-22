function cinematica_directa_mano
% =====================================================================
%  CINEMATICA DIRECTA DE UNA MANO PROTESICA
%  Cuatro dedos (indice, medio, anular, menique) + pulgar

clc; close all;

%% ----------------- Parametros geometricos de los dedos (mm) -----------
%            nombre     id          L0      L1       L2       L3      delta
ded(1) = struct('nombre','Indice', 'id','indice', 'L0',27.000,'L1',55.000,'L2',38.160,'L3',34.000,'delta',+33.3);
ded(2) = struct('nombre','Medio',  'id','medio',  'L0',27.000,'L1',69.000,'L2',45.500,'L3',33.020,'delta',  0.0);
ded(3) = struct('nombre','Anular', 'id','anular', 'L0',27.000,'L1',58.996,'L2',38.160,'L3',28.482,'delta',-33.3);
ded(4) = struct('nombre','Menique','id','menique','L0',27.000,'L1',45.510,'L2',33.210,'L3',22.000,'delta',-66.6);

%% ----------------- Angulos articulares de entrada ---------------------
% Por dedo: [theta1_deg (MCP), theta2_deg (PIP)].
% theta3 (DIP) se obtiene del acoplamiento DIP_from_PIP(theta2).
ang = [ 10 30 ;    % Indice
        10 35 ;    % Medio
        10 40 ;    % Anular
        10 45 ];   % Menique

%% ----------------- Figura ---------------------------------------------
figure('Color','w'); hold on; grid on; axis equal;
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('Cinematica directa - Mano protesica'); view(135,25);
colores = lines(5);

%% ----------------- Lazo sobre los cuatro dedos ------------------------
fprintf('--- Posicion de la yema (global) ---\n');
for i = 1:numel(ded)
    th1_deg = ang(i,1);
    th2_deg = ang(i,2);
    th3_deg = DIP_from_PIP(th2_deg, ded(i).id);   % acoplamiento real

    th1 = deg2rad(th1_deg);
    th2 = deg2rad(th2_deg);
    th3 = deg2rad(th3_deg);

    P = fk_dedo(ded(i), th1, th2, th3);           % 3x4: [MCP PIP DIP ACT]

    % Unir: centro de palma -> MCP -> PIP -> DIP -> ACT
    palma = [ded(i).delta; 0; 0];
    Pf = [palma, P];
    plot3(Pf(1,:), Pf(2,:), Pf(3,:), '-o', 'Color', colores(i,:), ...
          'LineWidth', 2, 'MarkerFaceColor', colores(i,:));
    text(P(1,end), P(2,end), P(3,end), ['  ' ded(i).nombre]);

    fprintf('%-8s ACT = [%8.2f, %8.2f, %8.2f] mm\n', ...
            ded(i).nombre, P(1,end), P(2,end), P(3,end));
end

%% ----------------- Pulgar ----------------------
pul = struct('L0',45.510,'L1',33.210,'L2',22.000);

tilt     = 50;                          % inclinacion de la base [deg]
pos_base = [26.127; -73.939; 0];        % traslacion de base [mm]

% Marco local de la base 
Yl = [ cosd(tilt);  sind(tilt); 0];
Xl = [ sind(tilt); -cosd(tilt); 0];
Zl = cross(Xl, Yl);  Zl = Zl / norm(Zl);
R_base = [Xl, Yl, Zl];

% Angulos de entrada del pulgar [grados]
th1_p = 0;                             % abduccion
th2_p = 25;                             % MCP (flexion)
th3_p = DIP_from_PIP(th2_p, 'pulgar');  % IP acoplado a MCP

Pp = fk_pulgar(pul, th1_p, th2_p, th3_p, pos_base, R_base);  % 3x4
plot3(Pp(1,:), Pp(2,:), Pp(3,:), '-s', 'Color', colores(5,:), ...
      'LineWidth', 2, 'MarkerFaceColor', colores(5,:));
text(Pp(1,end), Pp(2,end), Pp(3,end), '  Pulgar');

fprintf('%-8s ACT = [%8.2f, %8.2f, %8.2f] mm\n', ...
        'Pulgar', Pp(1,end), Pp(2,end), Pp(3,end));

legend([{ded.nombre}, {'Pulgar'}], 'Location','best');
end


% ======================================================
%  FUNCIONES LOCALES
% ======================================================

function P = fk_dedo(p, th1, th2, th3)
% Cinematica directa de un dedo (indice/medio/anular/menique).
% Entradas:  p   -> struct con L0,L1,L2,L3,delta
%            th1,th2,th3 -> angulos articulares en radianes
% Salida:    P   -> 3x4 con las columnas [MCP PIP DIP ACT] en marco global

    % Angulos acumulados
    phi1 = th1;
    phi2 = th1 + th2;
    phi3 = th1 + th2 + th3;

    % Plano local: fila 1 = z (sin), fila 2 = y (cos)
    P_MCP = [0;      p.L0];
    P_PIP = P_MCP + [p.L1*sin(phi1); p.L1*cos(phi1)];
    P_DIP = P_PIP + [p.L2*sin(phi2); p.L2*cos(phi2)];
    P_tip = P_DIP + [p.L3*sin(phi3); p.L3*cos(phi3)];

    local = [P_MCP, P_PIP, P_DIP, P_tip];   % 2x4 -> [z ; y]
    zc = local(1,:);
    yc = local(2,:);
    xc = p.delta * ones(1, size(local,2));  % desplazamiento lateral

    P = [xc; yc; zc];                        % [x ; y ; z]
end


function P = fk_pulgar(p, th1, th2, th3, pos_base, R_base)
% Cinematica directa del pulgar, extraida del codigo de cinematica
% inversa (resolverPulgar). Usa rotaciones de Rodrigues en el marco local
% y luego transforma al marco global con R_base y pos_base.
% Entradas:  p        -> struct con L0,L1,L2 (mm)
%            th1      -> abduccion [deg]
%            th2      -> MCP / flexion [deg]
%            th3      -> IP / acoplado a th2 [deg]
%            pos_base -> 3x1 traslacion de base (mm)
%            R_base   -> 3x3 rotacion de base
% Salida:    P        -> 3x4 con [base MCP IP tip] en marco global

    d_ext_loc = [0; 1; 0];   % direccion de extension en el plano local
    d_flex0   = [0; 0; 1];   % eje de flexion ANTES de abducir
    d_abd_ax  = [1; 0; 0];   % eje de abduccion

    % Abduccion: gira la falange Y TAMBIEN el eje de flexion.
    d_abd     = rotAxis(d_ext_loc, d_abd_ax, th1);
    d_flex_ax = rotAxis(d_flex0,   d_abd_ax, th1);   

    % Cadena en el marco local
    P1_loc = p.L0 * d_abd;                       % MCP
    dir1   = rotAxis(d_abd,  d_flex_ax, th2);
    P2_loc = P1_loc + p.L1 * dir1;               % IP
    dir2   = rotAxis(dir1,   d_flex_ax, th3);
    P3_loc = P2_loc + p.L2 * dir2;               % ACT

    % Transformacion al marco global
    P = [ pos_base, ...
          pos_base + R_base*P1_loc, ...
          pos_base + R_base*P2_loc, ...
          pos_base + R_base*P3_loc ];   % 3x4: [base MCP IP ACT]
end


function v_rot = rotAxis(v, ax, ang_deg)
% Rotacion de Rodrigues de un vector v alrededor del eje ax por ang_deg.
    ax  = ax / norm(ax);
    K   = [     0,    -ax(3),  ax(2); ...
            ax(3),       0,   -ax(1); ...
           -ax(2),   ax(1),       0 ];
    R     = eye(3) + sind(ang_deg)*K + (1 - cosd(ang_deg))*(K*K);
    v_rot = R * v;
    v_rot = v_rot / norm(v_rot);
end


function th4 = DIP_from_PIP(th3, dedo_id)
% Acoplamiento DIP-PIP del mecanismo de 4 barras (polinomio grado 4).
% Entrada th3 y salida th4 en GRADOS. 
    switch dedo_id
        case 'indice'
            th4 = (-0.00000016)*th3^4 + (0.00006721)*th3^3 ...
                + (-0.01062799)*th3^2 + (1.68304757)*th3 + (0.04984945);
        case 'medio'
            th4 = (0.00000107)*th3^4  + (-0.00012334)*th3^3 ...
                + (0.00087773)*th3^2  + (1.33867879)*th3 + (0.21523916);
        case 'anular'
            th4 = (-0.00000020)*th3^4 + (0.00007387)*th3^3 ...
                + (-0.01090612)*th3^2 + (1.68680914)*th3 + (0.04021311);
        case 'menique'
            th4 = (-0.00000010)*th3^4 + (0.00005985)*th3^3 ...
                + (-0.01029053)*th3^2 + (1.67216440)*th3 + (0.04004136);
        case 'pulgar'
            th4 = (0.00000075)*th3^4  + (-0.00008762)*th3^3 ...
                + (0.00106596)*th3^2  + (1.27148703)*th3 + (0.14418172);
        otherwise
            th4 = 0;
    end
    th4 = max(0, min(90, th4));
end