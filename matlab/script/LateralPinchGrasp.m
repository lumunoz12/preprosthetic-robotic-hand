function setpoints_deg = LateralPinchGrasp_setpoint(F_indice, F_pulgar, F_threshold, theta_real_deg)

    % PERSISTENTES
    persistent contacted_dedos contacted_pulgar theta_hold theta_grasp_calc stage

    if isempty(contacted_dedos)
        contacted_dedos  = false;
        contacted_pulgar = false;
        theta_hold       = zeros(11,1);
        theta_grasp_calc = zeros(11,1);
        stage            = 1;        % 1=dedos largos, 2=ABD pulgar, 3=IP pulgar
    end

    % PARAMETROS
    freeze_abd_on_contact = true;    % congelar ABD ademas del IP al tocar
    tol_deg               = 2.0;     % tolerancia de llegada por fase

    retroceso_ip_deg    = 0.0;
    retroceso_dedos_deg = 0.0;

    % Angulos objetivo iniciales (posicion de reposo)
    theta_0 = [(-143.6600000000000/-3.8151)-0.1556; ...
               -175.12670700588635+180-4.8; ...
                1.4516897910062629; ...
               -173.87060141711123+180-6.0; ...
                8.4987069032538063; ...
               -177.26207595048015+180-2.7; ...
                1.8689078585581695e-13; ...
               -177.71054335982637+180-2.1; ...
               10.54754775353744; ...
               -175.51191256300692+180-5.8; ...
               -164.83058974690246+180-14];

    % Calcular theta_grasp una sola vez
    if theta_grasp_calc(1) == 0.0 && theta_grasp_calc(2) == 0.0
        theta_grasp_calc = calcular_lateral_pinch_IK(theta_0);
    end
    theta_grasp = theta_grasp_calc;

    % Setpoint base: todo en reposo, salvo la junta base
    setpoints_deg     = theta_0;
    setpoints_deg(1)  = theta_grasp(1);

    % FASE 1: dedos largos (indice, medio, anular, menique)
    if stage >= 1
        setpoints_deg(2:9) = theta_grasp(2:9);
    end

    % Freeze dedos largos: latch del angulo real al tocar
    if ~contacted_dedos && (F_indice > F_threshold)
        contacted_dedos  = true;
        theta_hold(2:9)  = theta_real_deg(2:9) - retroceso_dedos_deg;
    end
    if contacted_dedos
        setpoints_deg(2:9) = theta_hold(2:9);
    end

    % Avance 1 -> 2: dedos largos tocaron o llegaron dentro de tolerancia
    if stage == 1
        llegaron_dedos = all(abs(theta_real_deg(2:9) - theta_grasp(2:9)) < tol_deg);
        if contacted_dedos || llegaron_dedos
            stage = 2;
        end
    end

    % FASE 2: ABD del pulgar
    if stage >= 2
        setpoints_deg(11) = theta_grasp(11);
    end

    % Avance 2 -> 3: ABD llego dentro de tolerancia
    if stage == 2
        if abs(theta_real_deg(11) - theta_grasp(11)) < tol_deg
            stage = 3;
        end
    end

    % FASE 3: IP del pulgar
    if stage >= 3
        setpoints_deg(10) = theta_grasp(10);
    end

    % Freeze pulgar (solo en fase 3): IP con retroceso; ABD sin retroceso
    if stage >= 3 && ~contacted_pulgar && (F_pulgar > F_threshold)
        contacted_pulgar = true;
        theta_hold(10)   = theta_real_deg(10) - retroceso_ip_deg;
        if freeze_abd_on_contact
            theta_hold(11) = theta_real_deg(11);
        end
    end
    if contacted_pulgar
        setpoints_deg(10) = theta_hold(10);
        if freeze_abd_on_contact
            setpoints_deg(11) = theta_hold(11);
        end
    end

end


% ============================================================
%  Calcula angulos objetivo mediante IK, con FK completa del pulgar.
%  La abduccion del pulgar es una rotacion pura alrededor del eje X
%  local (abd = atan2(z_loc, y_loc)).
% ============================================================
function theta_grasp = calcular_lateral_pinch_IK(theta_0)

    % Geometria dedos largos
    offset_i  =  33.300; L0_i  = 27.0; L1_i  = 55.000; L2_i  = 38.160; L3_i  = 34.000;
    offset_m  =   0.000; L0_m  = 27.0; L1_m  = 69.000; L2_m  = 45.500; L3_m  = 34.800;
    offset_a  = -33.300; L0_a  = 27.0; L1_a  = 61.500; L2_a  = 38.160; L3_a  = 31.700;
    offset_me = -66.600; L0_me = 27.0; L1_me = 55.000; L2_me = 33.210; L3_me = 27.300;

    % Geometria pulgar
    L0_p = 45.510;   % falange proximal
    L1_p = 33.210;   % falange media
    L2_p = 22.000;   % falange distal
    pos_base_pulgar = [26.127; -73.939; 0.0];
    tilt = 50.0;     % inclinacion del metacarpo del pulgar

    % PASO 1: FK indice (postura lider lateral)
    th2_i   = 75.0;
    th3_i   = 45.0;
    P_MCP_i = [L0_i; 0];
    [~, joints_i] = forwardFinger(th2_i, th3_i, L1_i, L2_i, L3_i, P_MCP_i, 1);

    base_i_3D = [offset_i; 0; 0];
    P_MCP_3D  = base_i_3D + [0; joints_i(1,1); joints_i(2,1)];
    P_PIP_3D  = base_i_3D + [0; joints_i(1,2); joints_i(2,2)];

    % Centro de la falange proximal: target de orientacion del pulgar
    factor_prox = 0.50;
    P_prox_mid  = P_MCP_3D + factor_prox * (P_PIP_3D - P_MCP_3D);

    fprintf('[LateralPinch] Target pulgar (mid falange proximal indice): X=%.2f Y=%.2f Z=%.2f mm\n', ...
            P_prox_mid(1), P_prox_mid(2), P_prox_mid(3));

    % PASO 2: ABD del pulgar (rotacion pura alrededor del eje X local)
    ip_pulgar_fijo = 28.0;  % IP objetivo (controlado en runtime por F_pulgar)

    Yl = [ cosd(tilt);  sind(tilt); 0];
    Xl = [ sind(tilt); -cosd(tilt); 0];
    Zl = cross(Xl, Yl);  Zl = Zl / norm(Zl);
    R_base_pulgar = [Xl, Yl, Zl];

    % Vector desde la base del pulgar hacia el target, en frame local
    V_abd   = P_prox_mid - pos_base_pulgar;
    V_local = R_base_pulgar' * V_abd;

    % Abduccion exacta y limitada al rango fisico del pulgar
    abd_pulgar_geo = atan2d(V_local(3), V_local(2));
    abd_pulgar = max(0.0, min(90.0, abd_pulgar_geo));

    % Validacion con FK analitica
    yema_pulgar_3D = fk_pulgar_y_jacobiano_eval(abd_pulgar, ip_pulgar_fijo, ...
                         pos_base_pulgar, tilt, L0_p, L1_p, L2_p);

    fprintf('[LateralPinch] ABD pulgar geo=%.2f deg -> limitado=%.2f deg\n', ...
            abd_pulgar_geo, abd_pulgar);
    fprintf('[LateralPinch] Yema pulgar (FK analitica): X=%.2f Y=%.2f Z=%.2f mm\n', ...
            yema_pulgar_3D(1), yema_pulgar_3D(2), yema_pulgar_3D(3));

    % PASO 3: IK dedos largos siguiendo la punta del indice
    punta_indice_3D = [offset_i; joints_i(1,4); joints_i(2,4)];

    [MCP_m,  PIP_m]  = ik_seguir_indice(punta_indice_3D, offset_m,  ...
                          L0_m,  L1_m,  L2_m,  L3_m,  2, th2_i, th3_i, 'Medio');
    [MCP_a,  PIP_a]  = ik_seguir_indice(punta_indice_3D, offset_a,  ...
                          L0_a,  L1_a,  L2_a,  L3_a,  3, th2_i, th3_i, 'Anular');
    [MCP_me, PIP_me] = ik_seguir_indice(punta_indice_3D, offset_me, ...
                          L0_me, L1_me, L2_me, L3_me, 4, th2_i, th3_i, 'Menique');

    fprintf('[LateralPinch] Medio   -> MCP=%.2f  PIP=%.2f\n', MCP_m,  PIP_m);
    fprintf('[LateralPinch] Anular  -> MCP=%.2f  PIP=%.2f\n', MCP_a,  PIP_a);
    fprintf('[LateralPinch] Menique -> MCP=%.2f  PIP=%.2f\n', MCP_me, PIP_me);

    % PASO 4: Construir theta_grasp
    theta_grasp     = theta_0;
    theta_grasp(2)  = th2_i;          % MCP indice (lider)
    theta_grasp(3)  = th3_i;          % PIP indice (lider)
    theta_grasp(4)  = MCP_m;          % MCP medio
    theta_grasp(5)  = PIP_m;          % PIP medio
    theta_grasp(6)  = MCP_a;          % MCP anular
    theta_grasp(7)  = PIP_a;          % PIP anular
    theta_grasp(8)  = MCP_me;         % MCP menique
    theta_grasp(9)  = PIP_me;         % PIP menique
    theta_grasp(10) = ip_pulgar_fijo; % IP pulgar
    theta_grasp(11) = abd_pulgar;     % ABD pulgar

end


% ============================================================
%  FK analitica del pulgar (solo evaluacion de posicion)
% ============================================================
function yema_3D = fk_pulgar_y_jacobiano_eval(abd, ip, pos_base, tilt, L0, L1, L2)
    [yema_3D, ~] = fk_pulgar_y_jacobiano(abd, ip, pos_base, tilt, L0, L1, L2);
end


% ============================================================
%  FK analitica del pulgar + Jacobiano
% ============================================================
function [yema_3D, J] = fk_pulgar_y_jacobiano(abd, ip, pos_base, tilt, L0, L1, L2)

    k = pi / 180.0;

    Yl = [ cosd(tilt);  sind(tilt); 0];
    Xl = [ sind(tilt); -cosd(tilt); 0];
    Zl = cross(Xl, Yl);  Zl = Zl / norm(Zl);
    R  = [Xl, Yl, Zl];

    th3      = DIP_from_PIP(ip, 5);     % DIP acoplado al IP (pulgar)
    dth3_dip = dDIP_dPIP(ip, 5);

    c1   = cosd(abd);
    s1   = sind(abd);

    % Posicion 2D en el plano de flexion (r = extension, h = flexion)
    r2d = L0 + L1*cosd(ip) + L2*cosd(ip + th3);
    h2d =      L1*sind(ip) + L2*sind(ip + th3);

    yema_loc = [-h2d; c1*r2d; s1*r2d];
    yema_3D  = pos_base + R * yema_loc;

    % Jacobiano analitico (derivadas respecto a abd e ip, en grados)
    dr_dip = k * (-L1*sind(ip) - L2*sind(ip + th3)*(1 + dth3_dip));
    dh_dip = k * ( L1*cosd(ip) + L2*cosd(ip + th3)*(1 + dth3_dip));

    dloc_dabd = k * [ 0; -s1*r2d; c1*r2d];
    dloc_dip  =     [-dh_dip; c1*dr_dip; s1*dr_dip];

    J = R * [dloc_dabd, dloc_dip];

end


% ============================================================
%  Rodrigues: rotacion de vector alrededor de un eje
% ============================================================
function v_rot = rotAxis(v, ax, ang_deg) %#ok<DEFNU>
    ax  = ax / norm(ax);
    K   = [     0,    -ax(3),  ax(2); ...
            ax(3),       0,   -ax(1); ...
           -ax(2),   ax(1),       0 ];
    R     = eye(3) + sind(ang_deg)*K + (1 - cosd(ang_deg))*(K*K);
    v_rot = R * v;
    v_rot = v_rot / norm(v_rot);
end


% ============================================================
%  IK dedo que sigue la punta del indice
% ============================================================
function [MCP, PIP] = ik_seguir_indice(punta_indice_3D, offset_d, ...
                          L0, L1, L2, L3, dedo_id, th2_ref, th3_ref, nombre)

    th4_ref = DIP_from_PIP(th3_ref, dedo_id);
    alcance = L0 ...
            + L1 * cosd(th2_ref) ...
            + L2 * cosd(th2_ref + th3_ref) ...
            + L3 * cosd(th2_ref + th3_ref + th4_ref);

    r_target = punta_indice_3D(2);
    z_target = punta_indice_3D(3);

    if r_target > alcance * 0.95
        factor   = (alcance * 0.90) / r_target;
        r_target = r_target * factor;
        z_target = z_target * factor;
        fprintf('[LateralPinch] %s: target ajustado a alcance real\n', nombre);
    end

    punta_3D   = [offset_d; r_target; z_target];
    [MCP, PIP] = ik_dedo(punta_3D, offset_d, L0, L1, L2, L3, dedo_id);

end


% ============================================================
%  IK dedo (wrapper 3D -> 2D sagital)
% ============================================================
function [th2_out, th3_out] = ik_dedo(punta_3D, ~, L0, L1, L2, L3, dedo_id)
    target_2d = [punta_3D(2); punta_3D(3)];
    P_MCP     = [L0; 0];
    [th2_out, th3_out] = ik_dedo_2D(target_2d, P_MCP, L1, L2, L3, dedo_id);
end


% ============================================================
%  IK 2D (Levenberg-Marquardt)
% ============================================================
function [th2_out, th3_out] = ik_dedo_2D(target_2d, P_MCP, L1, L2, L3, dedo_id)

    th2 = 20.0;
    th3 = 20.0;

    lambda   = 0.01;
    max_iter = 300;
    tol      = 0.15;

    for iter = 1:max_iter
        [p_tip, ~] = forwardFinger(th2, th3, L1, L2, L3, P_MCP, dedo_id);
        e = target_2d - p_tip;
        if norm(e) < tol
            break;
        end
        J   = jacobianFinger(th2, th3, L1, L2, L3, dedo_id);
        dth = (J'*J + lambda^2 * eye(2)) \ (J' * e);
        th2 = max(0.0, min(90.0, th2 + dth(1)));
        th3 = max(0.0, min(90.0, th3 + dth(2)));
    end

    th2_out = max(0.0, min(90.0, th2));
    th3_out = max(0.0, min(90.0, th3));

end


% ============================================================
%  FK 2D
% ============================================================
function [p_tip, joints] = forwardFinger(th2, th3, L1, L2, L3, P_MCP, dedo_id)
    th4    = DIP_from_PIP(th3, dedo_id);
    R2     = rot2d(th2);
    R3     = rot2d(th2 + th3);
    R4     = rot2d(th2 + th3 + th4);
    P1     = P_MCP;
    P2     = P1 + R2 * [L1; 0];
    P3     = P2 + R3 * [L2; 0];
    P4     = P3 + R4 * [L3; 0];
    p_tip  = P4;
    joints = [P1 P2 P3 P4];
end


% ============================================================
%  Jacobiano 2D
% ============================================================
function J = jacobianFinger(th2, th3, L1, L2, L3, dedo_id)
    k          = pi / 180.0;
    th4        = DIP_from_PIP(th3, dedo_id);
    dth4_dth3  = dDIP_dPIP(th3, dedo_id);
    phi2r      = deg2rad(th2);
    phi3r      = deg2rad(th2 + th3);
    phi4r      = deg2rad(th2 + th3 + th4);
    dphi4_dth3 = 1.0 + dth4_dth3;

    dr_dth2 = k * (-sin(phi2r)*L1 - sin(phi3r)*L2 - sin(phi4r)*L3);
    dz_dth2 = k * ( cos(phi2r)*L1 + cos(phi3r)*L2 + cos(phi4r)*L3);
    dr_dth3 = k * (-sin(phi3r)*L2 - sin(phi4r)*dphi4_dth3*L3);
    dz_dth3 = k * ( cos(phi3r)*L2 + cos(phi4r)*dphi4_dth3*L3);

    J = [dr_dth2 dr_dth3; dz_dth2 dz_dth3];
end


% ============================================================
%  DIP acoplado al PIP (casos 1-4 = dedos largos, caso 5 = pulgar)
% ============================================================
function th4 = DIP_from_PIP(th3, dedo_id)
    if dedo_id == 1
        th4 = (-0.00000016)*th3^4 + (0.00006721)*th3^3 ...
            + (-0.01062799)*th3^2 + (1.68304757)*th3 + (0.04984945);
    elseif dedo_id == 2
        th4 = (0.00000107)*th3^4  + (-0.00012334)*th3^3 ...
            + (0.00087773)*th3^2  + (1.33867879)*th3  + (0.21523916);
    elseif dedo_id == 3
        th4 = (-0.00000020)*th3^4 + (0.00007387)*th3^3 ...
            + (-0.01090612)*th3^2 + (1.68680914)*th3  + (0.04021311);
    elseif dedo_id == 4
        th4 = (-0.00000010)*th3^4 + (0.00005985)*th3^3 ...
            + (-0.01029053)*th3^2 + (1.67216440)*th3  + (0.04004136);
    elseif dedo_id == 5  % PULGAR
        th4 = (0.00000075)*th3^4  + (-0.00008762)*th3^3 ...
            + (0.00106596)*th3^2  + (1.27148703)*th3  + (0.14418172);
    else
        th4 = 0.0;
    end
    th4 = max(0.0, min(90.0, th4));
end


% ============================================================
%  Derivada DIP respecto al PIP (casos 1-4 + caso 5 = pulgar)
% ============================================================
function dth4 = dDIP_dPIP(th3, dedo_id)
    if dedo_id == 1
        dth4 = 4*(-0.00000016)*th3^3 + 3*(0.00006721)*th3^2 ...
             + 2*(-0.01062799)*th3   + (1.68304757);
    elseif dedo_id == 2
        dth4 = 4*(0.00000107)*th3^3  + 3*(-0.00012334)*th3^2 ...
             + 2*(0.00087773)*th3    + (1.33867879);
    elseif dedo_id == 3
        dth4 = 4*(-0.00000020)*th3^3 + 3*(0.00007387)*th3^2 ...
             + 2*(-0.01090612)*th3   + (1.68680914);
    elseif dedo_id == 4
        dth4 = 4*(-0.00000010)*th3^3 + 3*(0.00005985)*th3^2 ...
             + 2*(-0.01029053)*th3   + (1.67216440);
    elseif dedo_id == 5  % PULGAR
        dth4 = 4*(0.00000075)*th3^3  + 3*(-0.00008762)*th3^2 ...
             + 2*(0.00106596)*th3    + (1.27148703);
    else
        dth4 = 0.0;
    end
end


% ============================================================
%  Rotacion 2D
% ============================================================
function R = rot2d(th)
    R = [cosd(th) -sind(th); sind(th) cosd(th)];
end
