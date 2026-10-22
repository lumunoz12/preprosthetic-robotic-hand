function setpoints_deg = TipPinchGrasp_setpoint(F_indice, F_pulgar, F_threshold, theta_real_deg)

    % PERSISTENTES
    persistent contacted_indice contacted_pulgar theta_hold theta_grasp_calc stage

    if isempty(contacted_indice)
        contacted_indice  = false;
        contacted_pulgar  = false;
        theta_hold        = zeros(11,1);
        theta_grasp_calc  = zeros(11,1);
        stage             = 1;   % 1=ABD pulgar, 2=MCP pulgar, 3=indice+soporte
    end

    % PARAMETROS
    freeze_abd_on_contact = true;   % congelar ABD ademas del MCP al tocar el pulgar
    tol_deg               = 2.0;    % tolerancia de llegada por fase

    retroceso_mcp_pulgar_deg = 0.0;
    retroceso_dedos_deg      = 0.0;

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
        theta_grasp_calc = calcular_tip_pinch_IK(theta_0);
        fprintf('=== theta_grasp TipPinch (setpoint) ===\n');
        nombres = {'ABD_indice','MCP_indice','PIP_indice','MCP_medio','PIP_medio',...
                   'MCP_anular','PIP_anular','MCP_menique','PIP_menique','MCP_pulgar','ABD_pulgar'};
        for k = 1:11
            fprintf('  [%2d] %-14s  theta_0=%7.3f  grasp=%7.3f\n', ...
                    int32(k), nombres{k}, theta_0(k), theta_grasp_calc(k));
        end
    end
    theta_grasp = theta_grasp_calc;

    % Setpoint base: todo en reposo, salvo la junta base
    setpoints_deg    = theta_0;
    setpoints_deg(1) = theta_grasp(1);   % ABD indice, posicionada desde t=0

    % FASE 1: ABD del pulgar
    if stage >= 1
        setpoints_deg(11) = theta_grasp(11);
    end

    % Avance 1 -> 2: ABD llego dentro de tolerancia (no tiene sensor propio)
    if stage == 1
        if abs(theta_real_deg(11) - theta_grasp(11)) < tol_deg
            stage = 2;
        end
    end

    % FASE 2: MCP del pulgar
    if stage >= 2
        setpoints_deg(10) = theta_grasp(10);
    end

    % Freeze pulgar (solo en fase 2): MCP con retroceso; ABD sin retroceso
    if stage == 2 && ~contacted_pulgar && (F_pulgar > F_threshold)
        contacted_pulgar = true;
        theta_hold(10)   = theta_real_deg(10) - retroceso_mcp_pulgar_deg;
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

    % Avance 2 -> 3: pulgar (MCP) toco o llego dentro de tolerancia
    if stage == 2
        llego_mcp_pulgar = abs(theta_real_deg(10) - theta_grasp(10)) < tol_deg;
        if contacted_pulgar || llego_mcp_pulgar
            stage = 3;
        end
    end

    % FASE 3: indice + dedos de soporte (juntas 2-9)
    if stage >= 3
        setpoints_deg(2:9) = theta_grasp(2:9);
    end

    % Freeze indice + soporte: latch del angulo real al tocar
    if stage >= 3 && ~contacted_indice && (F_indice > F_threshold)
        contacted_indice = true;
        theta_hold(2:9)  = theta_real_deg(2:9) - retroceso_dedos_deg;
    end
    if contacted_indice
        setpoints_deg(2:9) = theta_hold(2:9);
    end

end


% ============================================================
%  Calcula angulos objetivo por interseccion en el plano X:
%  resuelve el encuentro yema-yema entre pulgar e indice sobre
%  el plano fijo del desfase lateral del indice.
% ============================================================
function theta_grasp = calcular_tip_pinch_IK(theta_0)
    % Geometria dedos largos
    offset_i  =  33.300; L0_i  = 27.0; L1_i  = 55.000; L2_i  = 38.160; L3_i  = 34.000;
    offset_m  =   0.000; L0_m  = 27.0; L1_m  = 69.000; L2_m  = 45.500; L3_m  = 34.800;
    offset_a  = -33.300; L0_a  = 27.0; L1_a  = 61.500; L2_a  = 38.160; L3_a  = 31.700;
    offset_me = -66.600; L0_me = 27.0; L1_me = 55.000; L2_me = 33.210; L3_me = 27.300;

    % Geometria pulgar
    L0_p = 45.510;
    L1_p = 33.210;
    L2_p = 22.000;
    pos_base_pulgar = [26.127; -73.939; 0.0];
    tilt = 50.0;
    X_objetivo_plano = offset_i;   % 33.3 mm
    P_MCP_i = [L0_i; 0];

    mcp_candidatos = 0.0:1.0:80.0;  % barrido de flexion MCP del pulgar
    gap_tol        = 1.0;           % mm: gap maximo para convergencia
    espesor_objeto_mm = 30.0;

    % Seleccion preferida: entre los que convergen, el de mayor Z
    best_Z      = -inf;
    sel_mcp     = 20.0;  sel_abd   = 15.0;
    sel_MCP_i   = 0.0;   sel_PIP_i = 0.0;
    sel_yema    = [offset_i; 0; 0];
    sel_gap     = inf;
    hay_conv    = false;

    % Respaldo: el de menor gap si nada converge dentro de gap_tol
    best_cost   = inf;
    fb_mcp      = 20.0;  fb_abd    = 15.0;
    fb_MCP_i    = 0.0;   fb_PIP_i  = 0.0;
    fb_yema     = [offset_i; 0; 0];
    encontrado  = false;

    for mcp_try = mcp_candidatos
        % Abduccion que lleva la yema del pulgar al plano X
        [abd_try, yema_try] = buscar_abd_pulgar_en_plano(...
            X_objetivo_plano, mcp_try, pos_base_pulgar, tilt, L0_p, L1_p, L2_p);

        % Si para este MCP el pulgar no toca el plano, se descarta
        if abs(yema_try(1) - X_objetivo_plano) > 1.0
            continue;
        end

        Y_t = yema_try(2);
        Z_t = yema_try(3);
        Y_t_obj = Y_t + espesor_objeto_mm;

        % El indice intenta alcanzar esa yema (IK sagital)
        [MCP_try, PIP_try] = ik_dedo_2D([Y_t_obj; Z_t], P_MCP_i, L1_i, L2_i, L3_i, 1);
        [p_tip_i, ~]       = forwardFinger(MCP_try, PIP_try, L1_i, L2_i, L3_i, P_MCP_i, 1);

        % Distancia entre yema del pulgar y yema del indice
        residual = norm([Y_t_obj; Z_t] - p_tip_i);

        % Respaldo (menor gap)
        if residual < best_cost
            best_cost  = residual;
            fb_mcp     = mcp_try;   fb_abd   = abd_try;
            fb_MCP_i   = MCP_try;   fb_PIP_i = PIP_try;
            fb_yema    = yema_try;
            encontrado = true;
        end

        % Seleccion preferida: convergen y maximizan Z
        if residual < gap_tol && Z_t > best_Z
            best_Z    = Z_t;
            sel_mcp   = mcp_try;    sel_abd   = abd_try;
            sel_MCP_i = MCP_try;    sel_PIP_i = PIP_try;
            sel_yema  = yema_try;
            sel_gap   = residual;
            hay_conv  = true;
        end
    end

    if hay_conv
        best_mcp   = sel_mcp;   best_abd   = sel_abd;
        best_MCP_i = sel_MCP_i; best_PIP_i = sel_PIP_i;
        best_yema  = sel_yema;
        best_cost  = sel_gap;
    elseif encontrado
        warning('[TipPinch] Ninguna postura converge bajo gap_tol=%.2f mm. Usando la de menor gap (%.3f mm).', ...
                gap_tol, best_cost);
        best_mcp   = fb_mcp;    best_abd   = fb_abd;
        best_MCP_i = fb_MCP_i;  best_PIP_i = fb_PIP_i;
        best_yema  = fb_yema;
    else
        error('[TipPinch] El pulgar no pudo alcanzar el plano X = %.1f en ningun MCP del barrido.', X_objetivo_plano);
    end

    % Postura final del pulgar y del indice en el encuentro
    mcp_pulgar  = best_mcp;        % MCP del pulgar (actuado)
    ip_pulgar   = DIP_from_PIP(mcp_pulgar, 5);  % IP acoplado (informativo)
    abd_pulgar  = best_abd;
    MCP_i       = best_MCP_i;
    PIP_i       = best_PIP_i;
    Y_encuentro = best_yema(2);
    Z_encuentro = best_yema(3);

    % Punta real del indice
    [~, joints_i]   = forwardFinger(MCP_i, PIP_i, L1_i, L2_i, L3_i, P_MCP_i, 1);
    punta_indice_3D = [offset_i; joints_i(1,4); joints_i(2,4)];

    fprintf('[TipPinch] Encuentro: MCP_pulgar=%.1f deg  (IP_acoplado=%.1f deg)  ABD_pulgar=%.2f deg  gap=%.3f mm\n', ...
            mcp_pulgar, ip_pulgar, abd_pulgar, best_cost);
    fprintf('[TipPinch] Punto de encuentro (mas alejado en +z): Y=%.2f mm  Z=%.2f mm\n', ...
            Y_encuentro, Z_encuentro);

    % IK dedos de soporte
    [MCP_m, PIP_m] = ik_dedo(punta_indice_3D, offset_m, L0_m, L1_m, L2_m, L3_m, 2);
    [MCP_a, PIP_a] = ik_dedo(punta_indice_3D, offset_a, L0_a, L1_a, L2_a, L3_a, 3);

    % Menique
    th4_ref    = DIP_from_PIP(PIP_i, 4);
    alcance_me = L0_me + L1_me * cosd(MCP_i) + L2_me * cosd(MCP_i + PIP_i) + L3_me * cosd(MCP_i + PIP_i + th4_ref);
    r_target_me = punta_indice_3D(2);
    z_target_me = punta_indice_3D(3);
    if r_target_me > alcance_me * 0.95
        factor      = (alcance_me * 0.90) / r_target_me;
        r_target_me = r_target_me * factor;
        z_target_me = z_target_me * factor;
    end
    punta_3D_me = [offset_me; r_target_me; z_target_me];
    [MCP_me, PIP_me] = ik_dedo(punta_3D_me, offset_me, L0_me, L1_me, L2_me, L3_me, 4);

    % Construir el vector final theta_grasp
    theta_grasp     = theta_0;
    theta_grasp(2)  = MCP_i;
    theta_grasp(3)  = PIP_i;
    theta_grasp(4)  = MCP_m;
    theta_grasp(5)  = PIP_m;
    theta_grasp(6)  = MCP_a;
    theta_grasp(7)  = PIP_a;
    theta_grasp(8)  = MCP_me;
    theta_grasp(9)  = PIP_me;
    theta_grasp(10) = mcp_pulgar;     % MCP del pulgar (actuado)
    theta_grasp(11) = abd_pulgar;
    fprintf('[Tip Pinch] Interseccion lograda en X = 33.3 mm\n');
    fprintf('            Coordenadas de contacto reales: Y = %.2f mm, Z = %.2f mm\n', Y_encuentro, Z_encuentro);
end


% ============================================================
%  Buscador numerico de abduccion del pulgar para plano fijo X
% ============================================================
function [abd_out, yema_3D] = buscar_abd_pulgar_en_plano(X_target, ip_fijo, pos_base, tilt, L0, L1, L2)
    abd = 15.0;
    max_iter = 200;
    tol = 0.05;
    learning_rate = 0.5;

    for iter = 1:max_iter
        [yema, J] = fk_pulgar_y_jacobiano(abd, ip_fijo, pos_base, tilt, L0, L1, L2);
        error_x = X_target - yema(1);

        if abs(error_x) < tol
            break;
        end

        if abs(J(1,1)) > 1e-4
            abd = abd + learning_rate * (error_x / J(1,1));
        else
            abd = abd + learning_rate * sign(error_x);
        end

        abd = max(0.0, min(60.0, abd));
    end

    [yema_3D, ~] = fk_pulgar_y_jacobiano(abd, ip_fijo, pos_base, tilt, L0, L1, L2);
    abd_out = abd;
end


% ============================================================
%  FK del pulgar + Jacobiano
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
%  IK 2D (Levenberg-Marquardt)
% ============================================================
function [th2_out, th3_out] = ik_dedo_2D(target_2d, P_MCP, L1, L2, L3, dedo_id)
    th2 = 20.0;
    th3 = 20.0;
    lambda    = 0.01;
    max_iter  = 300;
    tol       = 0.15;
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
%  IK dedo (wrapper 3D -> 2D sagital)
% ============================================================
function [th2_out, th3_out] = ik_dedo(punta_3D, ~, L0, L1, L2, L3, dedo_id)
    target_2d = [punta_3D(2); punta_3D(3)];
    P_MCP     = [L0; 0];
    [th2_out, th3_out] = ik_dedo_2D(target_2d, P_MCP, L1, L2, L3, dedo_id);
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
    elseif dedo_id == 5 % PULGAR
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
    elseif dedo_id == 5 % PULGAR
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