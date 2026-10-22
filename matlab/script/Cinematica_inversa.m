function Cinematica_inversa()
clc; close all;

% ============================================================
%  Cinematica Inversa - Mano Completa (5 Dedos)
%  Metodo: Jacobiano Analitico + Damped Least Squares
% ============================================================

Pobj_indice  = [33.3;  10; 60];
Pobj_medio   = [0;   90; 70];
Pobj_anular  = [-35; 80; 70];
Pobj_menique = [-70; 80; 70];
Pobj_pulgar  = [33.3; -2; 50];

% ============================================================
%  PARAMETROS DE CADA DEDO
% ============================================================
dedos = struct();

dedos(1).nombre  = 'Indice';
dedos(1).offset  =  33.300;
dedos(1).L0      =  27.000;
dedos(1).L1      =  55.000;
dedos(1).L2      =  38.160;
dedos(1).L3      =  34.0;
dedos(1).abduc   =  true;
dedos(1).Pobj    =  Pobj_indice;
dedos(1).color   =  [0.00 0.45 0.74];
dedos(1).dedo_id =  'indice';

dedos(2).nombre  = 'Medio';
dedos(2).offset  =  0.000;
dedos(2).L0      =  27.000;
dedos(2).L1      =  69.000;
dedos(2).L2      =  45.500;
dedos(2).L3      =  33.020;
dedos(2).abduc   =  false;
dedos(2).Pobj    =  Pobj_medio;
dedos(2).color   =  [0.47 0.67 0.19];
dedos(2).dedo_id =  'medio';

dedos(3).nombre  = 'Anular';
dedos(3).offset  = -33.300;
dedos(3).L0      =  27.000;
dedos(3).L1      =  58.996;
dedos(3).L2      =  38.160;
dedos(3).L3      =  28.482;
dedos(3).abduc   =  true;
dedos(3).Pobj    =  Pobj_anular;
dedos(3).color   =  [0.85 0.33 0.10];
dedos(3).dedo_id =  'anular';

dedos(4).nombre  = 'Menique';
dedos(4).offset  = -66.600;
dedos(4).L0      =  27.000;
dedos(4).L1      =  45.510;
dedos(4).L2      =  33.210;
dedos(4).L3      =  22.000;
dedos(4).abduc   =  true;
dedos(4).Pobj    =  Pobj_menique;
dedos(4).color   =  [0.64 0.08 0.18];
dedos(4).dedo_id =  'menique';

% ============================================================
%  PASO 1: Resolver indice primero para obtener th1_motor
% ============================================================
[th1_i, th2_i, th3_i, th4_i, th1_motor, joints_i, P3_i, tgt_i, ok_i, herr_i] = ...
    resolverDedo(dedos(1), NaN);

resultados(1).nombre     = dedos(1).nombre;
resultados(1).th1        = th1_i;
resultados(1).th2        = th2_i;
resultados(1).th3        = th3_i;
resultados(1).th4        = th4_i;
resultados(1).th1_motor  = th1_motor;
resultados(1).joints_sol = joints_i;
resultados(1).P3         = P3_i;
resultados(1).target     = tgt_i;
resultados(1).ok         = ok_i;
resultados(1).color      = dedos(1).color;
resultados(1).Pobj       = dedos(1).Pobj;
resultados(1).hist_err   = herr_i;

% ============================================================
%  PASO 2: Calcular abduccion real del anular desde th1_motor
% ============================================================
PIP_anular          = th1_motor + 180 - 360;
th1_anular_real     = abduc_motor_to_DIP_anular(PIP_anular);

fprintf('\n--- Calculo abduccion anular desde motor compartido ---\n');
fprintf('  th1_motor           = %8.4f deg\n', th1_motor);
fprintf('  PIP_anular          = %8.4f deg  (th1_motor + 180 - 360)\n', PIP_anular);
fprintf('  th1_anular_real     = %8.4f deg  (abduccion geometrica impuesta)\n', th1_anular_real);

B_an   = [dedos(3).offset; 0; 0];
V_an   = dedos(3).Pobj - B_an;
th1_anular_ideal = atan2d(V_an(2), V_an(1));
th1_anular_ideal = max(70, min(110, th1_anular_ideal));
fprintf('  th1_anular_ideal    = %8.4f deg  (abduccion geometrica libre)\n', th1_anular_ideal);
fprintf('  Diferencia          = %8.4f deg\n', th1_anular_real - th1_anular_ideal);

% ============================================================
%  PASO 3: Resolver medio (sin restriccion de abduccion)
% ============================================================
[th1_m, th2_m, th3_m, th4_m, ~, joints_m, P3_m, tgt_m, ok_m, herr_m] = ...
    resolverDedo(dedos(2), NaN);

resultados(2).nombre     = dedos(2).nombre;
resultados(2).th1        = th1_m;
resultados(2).th2        = th2_m;
resultados(2).th3        = th3_m;
resultados(2).th4        = th4_m;
resultados(2).th1_motor  = NaN;
resultados(2).joints_sol = joints_m;
resultados(2).P3         = P3_m;
resultados(2).target     = tgt_m;
resultados(2).ok         = ok_m;
resultados(2).color      = dedos(2).color;
resultados(2).Pobj       = dedos(2).Pobj;
resultados(2).hist_err   = herr_m;

% ============================================================
%  PASO 4: Resolver anular con th1 impuesto por el motor
% ============================================================
[th1_a, th2_a, th3_a, th4_a, ~, joints_a, P3_a, tgt_a, ok_a, herr_a] = ...
    resolverDedo(dedos(3), th1_anular_real);

resultados(3).nombre     = dedos(3).nombre;
resultados(3).th1        = th1_a;
resultados(3).th2        = th2_a;
resultados(3).th3        = th3_a;
resultados(3).th4        = th4_a;
resultados(3).th1_motor  = NaN;
resultados(3).joints_sol = joints_a;
resultados(3).P3         = P3_a;
resultados(3).target     = tgt_a;
resultados(3).ok         = ok_a;
resultados(3).color      = dedos(3).color;
resultados(3).Pobj       = dedos(3).Pobj;
resultados(3).hist_err   = herr_a;

% ============================================================
%  PASO 5: Resolver menique (sin restriccion)
% ============================================================
[th1_me, th2_me, th3_me, th4_me, ~, joints_me, P3_me, tgt_me, ok_me, herr_me] = ...
    resolverDedo(dedos(4), NaN);

resultados(4).nombre     = dedos(4).nombre;
resultados(4).th1        = th1_me;
resultados(4).th2        = th2_me;
resultados(4).th3        = th3_me;
resultados(4).th4        = th4_me;
resultados(4).th1_motor  = NaN;
resultados(4).joints_sol = joints_me;
resultados(4).P3         = P3_me;
resultados(4).target     = tgt_me;
resultados(4).ok         = ok_me;
resultados(4).color      = dedos(4).color;
resultados(4).Pobj       = dedos(4).Pobj;
resultados(4).hist_err   = herr_me;

% ============================================================
%  RESOLVER PULGAR 
% ============================================================
res_pulgar = resolverPulgar(Pobj_pulgar);

% ============================================================
%  IMPRIMIR RESULTADOS
% ============================================================
fprintf('\n=============================================================\n');
fprintf('   CINEMATICA INVERSA - MANO COMPLETA (Jacobiano + DLS)     \n');
fprintf('=============================================================\n');

for d = 1:4
    r  = resultados(d);
    dd = dedos(d);
    fprintf('\n--- Dedo %s ---\n', r.nombre);
    if r.ok
        fprintf('  Estado                : Convergencia OK\n');
    else
        fprintf('  Estado                : No convergio\n');
    end

    if strcmp(dd.dedo_id, 'indice')
        fprintf('  th1 (abduc. geom.)    = %8.4f deg  [geometrica libre]\n',   r.th1);
        fprintf('  th1 (motor abduc.)    = %8.4f deg  [motor fisico, PRIORIDAD]\n', r.th1_motor);
    elseif strcmp(dd.dedo_id, 'anular')
        fprintf('  th1 (abduc. impuesta) = %8.4f deg  [impuesta por motor]\n',  r.th1);
        fprintf('  th1 (abduc. ideal)    = %8.4f deg  [geometrica libre]\n',     th1_anular_ideal);
    elseif dd.abduc
        fprintf('  th1 (abduccion)       = %8.4f deg\n', r.th1);
    else
        fprintf('  th1 (abduccion)       =  90.0000 deg  [FIJO]\n');
    end

    fprintf('  th2 (MCP)             = %8.4f deg\n', r.th2);
    fprintf('  th3 (PIP)             = %8.4f deg\n', r.th3);
    fprintf('  th4 (DIP acop.)       = %8.4f deg\n', r.th4);
    tip = r.joints_sol(4,:);
    fprintf('  Error punta           = %8.4f mm\n', norm(tip - r.target'));
end

fprintf('\n--- Pulgar ---\n');
if res_pulgar.ok
    fprintf('  Estado               : Convergencia OK\n');
else
    fprintf('  Estado               : No convergio\n');
end
fprintf('  th1 (abduccion)      = %8.4f deg  [exacta: asin(z_loc/Ltot)]\n', res_pulgar.th1);
fprintf('  th2 (MCP = PIP_pol)  = %8.4f deg\n', res_pulgar.th2);
fprintf('  th3 (IP  = DIP_pol)  = %8.4f deg  [acoplado desde th2]\n', res_pulgar.th3);
fprintf('  Error punta (3D)     = %8.4f mm\n',  res_pulgar.error);
fprintf('\n=============================================================\n\n');

% ============================================================
%  GRAFICAS
% ============================================================
grafica3D(resultados, dedos, res_pulgar);

end


%% ============================================================
%  resolverDedo
% ============================================================
function [th1, th2, th3, th4, th1_motor, joints_sol, P3, target, ok, hist_err] = ...
         resolverDedo(dd, th1_forzado)

    Pobj = dd.Pobj;
    B    = [dd.offset; 0; 0];
    L0 = dd.L0;  L1 = dd.L1;  L2 = dd.L2;  L3 = dd.L3;

    V  = Pobj - B;
    dx = V(1);  dy = V(2);  dz = V(3);

    % th1: abduccion — geometrica o impuesta
    if isnan(th1_forzado)
        if dd.abduc
            th1 = atan2d(dy, dx);
            th1 = max(70, min(110, th1));
        else
            th1 = 90;
        end
    else
        % Abduccion impuesta por el motor compartido
        th1 = th1_forzado;
    end

    % th1_motor: solo para el indice
    if strcmp(dd.dedo_id, 'indice')
        th1_motor = abduc_DIP_to_motor_indice(th1);
    else
        th1_motor = NaN;
    end

    r_target = sqrt(dx^2 + dy^2);
    target   = [r_target; dz];
    P_MCP    = [L0; 0];

    th2 = 20;
    th3 = 30;

    alpha    = 1.0;
    lambda   = 0.01;
    max_iter = 200;
    tol      = 0.1;

    hist_err = zeros(1, max_iter);
    ok       = false;

    for iter = 1:max_iter
        [p_tip, ~] = forwardFinger(th2, th3, L1, L2, L3, P_MCP, dd.dedo_id);
        e   = target - p_tip;
        err = norm(e);
        hist_err(iter) = err;
        if err < tol
            ok = true;
            hist_err = hist_err(1:iter);
            break;
        end
        J        = jacobianFinger(th2, th3, L1, L2, L3, dd.dedo_id);
        delta_th = (J'*J + lambda^2 * eye(2)) \ (J' * e);
        th2 = th2 + alpha * delta_th(1);
        th3 = th3 + alpha * delta_th(2);
        th2 = max(0, min(90, th2));
        th3 = max(0, min(90, th3));
    end

    if ~ok
        hist_err = hist_err(1:max_iter);
    end

    [~, joints_2d] = forwardFinger(th2, th3, L1, L2, L3, P_MCP, dd.dedo_id);
    th4 = DIP_from_PIP(th3, dd.dedo_id);

    joints_sol = joints_2d';

    d_rad = [cosd(th1); sind(th1); 0];
    d_z   = [0; 0; 1];

    all_r = [0, joints_2d(1,:)];
    all_z = [0, joints_2d(2,:)];

    P3 = zeros(length(all_r), 3);
    for k = 1:length(all_r)
        P3(k,:) = (B + all_r(k)*d_rad + all_z(k)*d_z)';
    end

end


%% ============================================================
%  POLINOMIOS ABDUCCION INDICE
% ============================================================
function abd = abduc_motor_to_DIP_indice(th_motor)
    abd = (0.00000016)*th_motor^4 ...
        + (0.00003357)*th_motor^3 ...
        + (-0.01022825)*th_motor^2 ...
        + (0.09093474)*th_motor ...
        + (98.43718719);
end

function th_motor = abduc_DIP_to_motor_indice(abd)
    th_motor = (-0.00025462)*abd^4 ...
             + (0.08320608)*abd^3 ...
             + (-10.17318753)*abd^2 ...
             + (549.85457309)*abd ...
             + (-10998.37224391);
end


%% ============================================================
%  POLINOMIOS ABDUCCION ANULAR
% ============================================================
function abd = abduc_motor_to_DIP_anular(pip_anular)
    abd = (0.00000009)*pip_anular^4 ...
        + (0.00003262)*pip_anular^3 ...
        + (0.00706480)*pip_anular^2 ...
        + (1.42181200)*pip_anular ...
        + (205.76920974);
end

function pip_anular = abduc_DIP_to_PIP_anular(abd) %#ok<DEFNU>
    pip_anular = (-0.00049101)*abd^4 ...
               + (0.19553608)*abd^3 ...
               + (-29.17671625)*abd^2 ...
               + (1935.36418359)*abd ...
               + (-48324.58012158);
end


%% ============================================================
%  resolverPulgar
% ============================================================
function res = resolverPulgar(Pobj)

    L0   = 45.510;
    L1   = 33.210;
    L2   = 22.000;

    pos_base = [26.127; -73.939; 0];
    tilt     = 50;

    % ---- Marco local de la base ----
    Yl = [ cosd(tilt);  sind(tilt); 0];
    Xl = [ sind(tilt); -cosd(tilt); 0];
    Zl = cross(Xl, Yl);  Zl = Zl / norm(Zl);
    R_base = [Xl, Yl, Zl];

    % ---- Objetivo en el marco local ----
    delta       = Pobj - pos_base;
    delta_local = R_base' * delta;
    dl_x = delta_local(1);
    dl_y = delta_local(2);
    dl_z = delta_local(3);

    % ---- th1 (abduccion): descomposicion EXACTA del modelo directo ----
    %  La cadena vive en el plano {x_local, d_abd}, con d_abd = [0; cos th1; sin th1].
    %  La punta local vale [ -q ; p*cos th1 ; p*sin th1 ]  ->  th1 = atan2(dl_z, dl_y).
    th1 = atan2d(dl_z, dl_y);
    th1 = max(0, min(90, th1));

    % ---- Objetivo 2D en el plano de flexion ----
    %  p = radio en el plano y-z (independiente de th1);  h = -x_local
    r_t    = sqrt(dl_y^2 + dl_z^2);
    h_t    = -dl_x;
    target = [r_t; h_t];

    P_MCP = [L0; 0];
    th2   = 20;

    alpha    = 1.0;
    lambda   = 0.01;
    max_iter = 200;
    tol      = 0.1;

    hist_err = zeros(1, max_iter);
    ok       = false;

    for iter = 1:max_iter
        th3 = DIP_from_PIP(th2, 'pulgar');
        [p_tip, ~] = forwardPulgar(th2, th3, L1, L2, P_MCP);
        e   = target - p_tip;
        err = norm(e);
        hist_err(iter) = err;
        if err < tol
            ok = true;
            hist_err = hist_err(1:iter);
            break;
        end
        J         = jacobianPulgar(th2, th3, L1, L2);
        delta_th2 = (J' * e) / (J'*J + lambda^2);
        th2 = th2 + alpha * delta_th2;
        th2 = max(0, min(90, th2));
    end

    if ~ok
        hist_err = hist_err(1:max_iter);
    end

    th3 = DIP_from_PIP(th2, 'pulgar');
    [~, joints_2d] = forwardPulgar(th2, th3, L1, L2, P_MCP);

    % ---- Reconstruccion 3D IDENTICA al modelo directo (CD) ----
    %  El eje de flexion ROTA con la abduccion, igual que fk_pulgar.
    d_ext_loc = [0; 1; 0];
    d_flex0   = [0; 0; 1];
    d_abd_ax  = [1; 0; 0];

    d_abd     = rotAxis(d_ext_loc, d_abd_ax, th1);
    d_flex_ax = rotAxis(d_flex0,   d_abd_ax, th1);   % <-- clave: rota con th1

    P1_loc = L0 * d_abd;
    dir1   = rotAxis(d_abd,  d_flex_ax, th2);
    P2_loc = P1_loc + L1 * dir1;
    dir2   = rotAxis(dir1,   d_flex_ax, th3);
    P3_loc = P2_loc + L2 * dir2;

    P3 = [ pos_base, ...
           pos_base + R_base*P1_loc, ...
           pos_base + R_base*P2_loc, ...
           pos_base + R_base*P3_loc ]';

    tip3d = P3(4, :)';

    res.nombre     = 'Pulgar';
    res.th1        = th1;
    res.th2        = th2;
    res.th3        = th3;
    res.joints_sol = joints_2d';
    res.P3         = P3;
    res.target     = target;
    res.ok         = ok;
    res.Pobj       = Pobj;
    res.color      = [0.00 0.75 0.75];
    res.error      = norm(tip3d - Pobj);   % error 3D real punta-objetivo
    res.hist_err   = hist_err;
    res.L0         = L0;
    res.L1         = L1;
    res.L2         = L2;
    res.pos_base   = pos_base;

end

%% ============================================================
%  forwardPulgar  (plano de flexion local: r = extension, h = flexion)
% ============================================================
function [p_tip, joints] = forwardPulgar(th2, th3, L1, L2, P_MCP)
    R2 = rot(th2);
    R3 = rot(th2 + th3);
    P1 = P_MCP;
    P2 = P1 + R2 * [L1; 0];
    P3 = P2 + R3 * [L2; 0];
    p_tip  = P3;
    joints = [P1, P2, P3];
end


%% ============================================================
%  jacobianPulgar
% ============================================================
function J = jacobianPulgar(th2, th3, L1, L2)
    k          = pi/180;
    dth3_dth2  = dDIP_dPIP(th2, 'pulgar');
    phi2r      = deg2rad(th2);
    phi3r      = deg2rad(th2 + th3);
    dphi3_dth2 = 1 + dth3_dth2;
    dr_dth2    = k * (-sin(phi2r)*L1 - sin(phi3r)*dphi3_dth2*L2);
    dz_dth2    = k * ( cos(phi2r)*L1 + cos(phi3r)*dphi3_dth2*L2);
    J = [dr_dth2; dz_dth2];
end


%% ============================================================
%  forwardFinger
% ============================================================
function [p_tip, joints] = forwardFinger(th2, th3, L1, L2, L3, P_MCP, dedo_id)
    th4 = DIP_from_PIP(th3, dedo_id);
    R2  = rot(th2);
    R3  = rot(th2 + th3);
    R4  = rot(th2 + th3 + th4);
    P1  = P_MCP;
    P2  = P1 + R2 * [L1; 0];
    P3  = P2 + R3 * [L2; 0];
    P4  = P3 + R4 * [L3; 0];
    p_tip  = P4;
    joints = [P1 P2 P3 P4];
end


%% ============================================================
%  jacobianFinger
% ============================================================
function J = jacobianFinger(th2, th3, L1, L2, L3, dedo_id)
    k         = pi/180;
    th4       = DIP_from_PIP(th3, dedo_id);
    dth4_dth3 = dDIP_dPIP(th3, dedo_id);
    phi2r = deg2rad(th2);
    phi3r = deg2rad(th2 + th3);
    phi4r = deg2rad(th2 + th3 + th4);
    dphi4_dth2 = 1;
    dphi4_dth3 = 1 + dth4_dth3;
    dr_dth2 = k*(-sin(phi2r)*L1 - sin(phi3r)*L2 - sin(phi4r)*dphi4_dth2*L3);
    dz_dth2 = k*( cos(phi2r)*L1 + cos(phi3r)*L2 + cos(phi4r)*dphi4_dth2*L3);
    dr_dth3 = k*(-sin(phi3r)*L2 - sin(phi4r)*dphi4_dth3*L3);
    dz_dth3 = k*( cos(phi3r)*L2 + cos(phi4r)*dphi4_dth3*L3);
    J = [dr_dth2  dr_dth3;
         dz_dth2  dz_dth3];
end


%% ============================================================
%  DIP_from_PIP
% ============================================================
function th4 = DIP_from_PIP(th3, dedo_id)
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


%% ============================================================
%  dDIP_dPIP
% ============================================================
function dth4 = dDIP_dPIP(th3, dedo_id)
    switch dedo_id
        case 'indice'
            dth4 = 4*(-0.00000016)*th3^3 + 3*(0.00006721)*th3^2 ...
                 + 2*(-0.01062799)*th3   + (1.68304757);
        case 'medio'
            dth4 = 4*(0.00000107)*th3^3  + 3*(-0.00012334)*th3^2 ...
                 + 2*(0.00087773)*th3    + (1.33867879);
        case 'anular'
            dth4 = 4*(-0.00000020)*th3^3 + 3*(0.00007387)*th3^2 ...
                 + 2*(-0.01090612)*th3   + (1.68680914);
        case 'menique'
            dth4 = 4*(-0.00000010)*th3^3 + 3*(0.00005985)*th3^2 ...
                 + 2*(-0.01029053)*th3   + (1.67216440);
        case 'pulgar'
            dth4 = 4*(0.00000075)*th3^3  + 3*(-0.00008762)*th3^2 ...
                 + 2*(0.00106596)*th3    + (1.27148703);
        otherwise
            dth4 = 0;
    end
end


%% ============================================================
%  rot  (rotacion 2D)
% ============================================================
function R = rot(th)
    R = [cosd(th) -sind(th);
         sind(th)  cosd(th)];
end


%% ============================================================
%  rotAxis — RODRIGUES (misma funcion que el workspace)
%  Devuelve vector unitario
% ============================================================
function v_rot = rotAxis(v, ax, ang_deg)
    ax  = ax / norm(ax);
    K   = [     0,    -ax(3),  ax(2); ...
            ax(3),       0,   -ax(1); ...
           -ax(2),   ax(1),       0 ];
    R     = eye(3) + sind(ang_deg)*K + (1 - cosd(ang_deg))*(K*K);
    v_rot = R * v;
    v_rot = v_rot / norm(v_rot);
end


%% ============================================================
%  grafica3D
% ============================================================
function grafica3D(resultados, dedos, res_pulgar)

    figure('Name', 'Mano Completa - Jacobiano 3D', ...
           'Color', 'w', 'Position', [150 150 900 650]);
    hold on; grid on; axis equal;
    xlabel('X [mm]', 'FontSize', 12);
    ylabel('Y [mm]', 'FontSize', 12);
    zlabel('Z [mm]', 'FontSize', 12);
    title('Mano Completa — Jacobiano Analitico + DLS', ...
          'FontSize', 14, 'FontWeight', 'bold');
    view(135, 30);

    for d = 1:4
        r   = resultados(d);
        dd  = dedos(d);
        P3  = r.P3;
        col = dd.color;

        plot3(P3(1:2,1), P3(1:2,2), P3(1:2,3), ...
              'k-o', 'LineWidth', 3, 'MarkerSize', 5, ...
              'HandleVisibility', 'off');
        plot3(P3(2:5,1), P3(2:5,2), P3(2:5,3), ...
              '-o', 'Color', col, 'LineWidth', 2.5, 'MarkerSize', 6, ...
              'DisplayName', sprintf('Dedo %s', dd.nombre));
        plot3(dd.Pobj(1), dd.Pobj(2), dd.Pobj(3), 'x', ...
              'Color', col, 'MarkerSize', 12, 'LineWidth', 2.5, ...
              'HandleVisibility', 'off');
        text(P3(5,1)+1, P3(5,2)+1, P3(5,3)+2, ...
             dd.nombre, 'FontSize', 8, 'FontWeight', 'bold', 'Color', col);
    end

    Pp  = res_pulgar.P3;
    col = res_pulgar.color;

    plot3(Pp(1:2,1), Pp(1:2,2), Pp(1:2,3), ...
          'k-o', 'LineWidth', 3, 'MarkerSize', 5, ...
          'HandleVisibility', 'off');
    plot3(Pp(2:4,1), Pp(2:4,2), Pp(2:4,3), ...
          '-o', 'Color', col, 'LineWidth', 2.5, 'MarkerSize', 6, ...
          'DisplayName', 'Pulgar');
    plot3(res_pulgar.Pobj(1), res_pulgar.Pobj(2), res_pulgar.Pobj(3), 'x', ...
          'Color', col, 'MarkerSize', 12, 'LineWidth', 2.5, ...
          'HandleVisibility', 'off');
    text(Pp(4,1)+1, Pp(4,2)+1, Pp(4,3)+2, ...
         'Pulgar', 'FontSize', 8, 'FontWeight', 'bold', 'Color', col);

    bx = arrayfun(@(d) d.P3(1,1), resultados);
    by = arrayfun(@(d) d.P3(1,2), resultados);
    bz = arrayfun(@(d) d.P3(1,3), resultados);
    fill3([bx bx(1)], [by by(1)], [bz bz(1)], ...
          [0.94 0.87 0.80], 'FaceAlpha', 0.35, ...
          'EdgeColor', [0.6 0.5 0.4], 'LineWidth', 1.5, ...
          'DisplayName', 'Palma');

    legend('Location', 'best', 'FontSize', 9);
end