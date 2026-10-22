function setpoints_deg = CylindricalGrasp_setpoint(F_dedos, F_pulgar, F_threshold, theta_real_deg)
    % ============================================================
    % AGARRE CILINDRICO  (SETPOINT + STAGES)  -- targets por IK real
    % ============================================================

    % =========================
    % PERSISTENTES
    % =========================
    persistent contacted_dedos contacted_pulgar theta_hold theta_grasp_calc stage

    if isempty(contacted_dedos)
        contacted_dedos   = false;
        contacted_pulgar  = false;
        theta_hold        = zeros(11,1);
        theta_grasp_calc  = zeros(11,1);
        stage             = 1;   % 1=ABD pulgar, 2=MCP indice+pulgar, 3=PIP indice
    end

    % =========================
    % PARAMETROS
    % =========================
    freeze_abd_on_contact  = true;   % congelar tambien la ABD del pulgar al tocar
    tol_deg                = 2.0;    % tolerancia para dar una fase por "llegada"

    % Referencia manual de flexion del PIP del indice.
    pip_ref_indice_deg = 48.0;   % [deg]  (max flexion)

    % Fraccion de cierre que copian los seguidores:
    seguir_indice_por_real = true;

    % Retroceso anti-overshoot (cierre va de MENOR a MAYOR angulo; retraer = restar)
    retroceso_mcp_pulgar_deg = 0.0;
    retroceso_dedos_deg      = 0.0;

    % ============================================================
    % CILINDRO OBJETIVO  (>>> AJUSTAR a tu escena <<<)
    % ============================================================
    cyl_yc = -35.0;   % posicion del eje en Y [mm]
    cyl_zc =  50.0;   % posicion del eje en Z [mm]
    cyl_R  =  38.0;   % radio del cilindro   [mm]

    % Pulgar: FALLBACK hardcodeado, solo se usa si el cilindro queda fuera
    % del alcance del pulgar (entonces no se puede resolver su contacto).
    abd_pulgar_obj = 75.0;
    mcp_pulgar_obj = 31.3;

    % ============================================================
    % ANGULOS DE REPOSO (mano abierta)
    % ============================================================
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

    % ============================================================
    % CALCULAR theta_grasp UNA SOLA VEZ (IK por dedo sobre el cilindro)
    % ============================================================
    if theta_grasp_calc(1) == 0.0 && theta_grasp_calc(2) == 0.0
        theta_grasp_calc = calcular_cilindro_target(theta_0, ...
            cyl_yc, cyl_zc, cyl_R, abd_pulgar_obj, mcp_pulgar_obj);
        fprintf('=== theta_grasp Cilindrico (IK por dedo, cilindro y_c=%.1f z_c=%.1f R=%.1f) ===\n', ...
                cyl_yc, cyl_zc, cyl_R);
        nombres = {'ABD_indice','MCP_indice','PIP_indice','MCP_medio','PIP_medio',...
                   'MCP_anular','PIP_anular','MCP_menique','PIP_menique','MCP_pulgar','ABD_pulgar'};
        for k = 1:11
            fprintf('  [%2d] %-14s  theta_0=%7.3f  grasp=%7.3f\n', ...
                    int32(k), nombres{k}, theta_0(k), theta_grasp_calc(k));
        end
        fprintf('  (Nota: PIP_indice usa referencia manual = %.1f, la IK[3] es solo informativa)\n', ...
                pip_ref_indice_deg);
    end
    theta_grasp = theta_grasp_calc;

    % =========================
    % SETPOINT BASE: TODO EN REPOSO, salvo la junta base (1)
    % =========================
    setpoints_deg    = theta_0;
    setpoints_deg(1) = theta_grasp(1);   % ABD indice en reposo

    % ============================================================
    % FASE 1: ABD DEL PULGAR (junta 11)
    % ============================================================
    if stage >= 1
        setpoints_deg(11) = theta_grasp(11);
    end
    if stage == 1
        if abs(theta_real_deg(11) - theta_grasp(11)) < tol_deg
            stage = 2;
        end
    end

    % ============================================================
    % FASE 2: MCP DEL INDICE (junta 2) + MCP DEL PULGAR (junta 10)
    % ============================================================
    if stage >= 2
        setpoints_deg(2)  = theta_grasp(2);    % MCP indice
        setpoints_deg(10) = theta_grasp(10);   % MCP pulgar
    end

    % Freeze pulgar por su FSR (independiente del indice)
    if stage >= 2 && ~contacted_pulgar && (F_pulgar > F_threshold)
        contacted_pulgar = true;
        theta_hold(10)   = theta_real_deg(10) - retroceso_mcp_pulgar_deg;
        if freeze_abd_on_contact
            theta_hold(11) = theta_real_deg(11);
        end
        fprintf('[Cilindro] Pulgar detenido por contacto (F=%.2f N)\n', F_pulgar);
    end
    if contacted_pulgar
        setpoints_deg(10) = theta_hold(10);
        if freeze_abd_on_contact
            setpoints_deg(11) = theta_hold(11);
        end
    end

    % Avance 2 -> 3
    if stage == 2
        llego_mcp_indice = abs(theta_real_deg(2) - theta_grasp(2)) < tol_deg;
        if contacted_dedos || llego_mcp_indice
            stage = 3;
        end
    end

    % ============================================================
    % FASE 3: PIP DEL INDICE (junta 3)  -- REFERENCIA MANUAL
    % ============================================================
    if stage >= 3
        setpoints_deg(3) = pip_ref_indice_deg;   % PIP indice: referencia manual, cierra hasta FSR
    end

    % Freeze del INDICE por su FSR (F_dedos): congela MCP(2) y PIP(3)
    if stage >= 2 && ~contacted_dedos && (F_dedos > F_threshold)
        contacted_dedos = true;
        theta_hold(2) = theta_real_deg(2) - retroceso_dedos_deg;
        theta_hold(3) = theta_real_deg(3) - retroceso_dedos_deg;   % congela el PIP donde toco
        fprintf('[Cilindro] Indice detenido por contacto (F=%.2f N)\n', F_dedos);
    end
    if contacted_dedos
        setpoints_deg(2) = theta_hold(2);
        setpoints_deg(3) = theta_hold(3);
    end

    % ============================================================
    % DEDOS SEGUIDORES: medio (4,5), anular (6,7), menique (8,9)
    % ============================================================

    % --- MCP: setpoint del indice (mueve los cuatro MCP juntos) ---
    ref_mcp = setpoints_deg(2);   % theta_grasp(2) en Fase 2, o theta_hold(2) si congelado
    den_mcp = theta_grasp(2) - theta_0(2);   if abs(den_mcp) < 1e-6, den_mcp = 1e-6; end
    s_mcp   = min(1, max(0, (ref_mcp - theta_0(2)) / den_mcp));

    % --- PIP del indice: angulo de referencia para medir su avance de punta ---
    if contacted_dedos
        pip_indice_ref = theta_hold(3);      % congelado -> se detienen todos
    elseif seguir_indice_por_real
        pip_indice_ref = theta_real_deg(3);  % sigue el curl REAL del indice
    else
        pip_indice_ref = setpoints_deg(3);   % (no recomendado) sigue el setpoint
    end

    % --- Avance fraccional de la punta del indice por su PIP (MCP en objetivo) ---
    [iL0,iL1,iL2,iL3,iid] = finger_geo(1);          % 1 = indice
    P_MCP_i = [iL0; 0];
    tip_a = forwardFinger(theta_grasp(2), theta_0(3),     iL1,iL2,iL3,P_MCP_i,iid);
    tip_b = forwardFinger(theta_grasp(2), theta_grasp(3), iL1,iL2,iL3,P_MCP_i,iid);
    tip_n = forwardFinger(theta_grasp(2), pip_indice_ref, iL1,iL2,iL3,P_MCP_i,iid);
    den_tip = norm(tip_b - tip_a);   if den_tip < 1e-6, den_tip = 1e-6; end
    f_pip = min(1, max(0, norm(tip_n - tip_a) / den_tip));

    % --- Seguidores: MCP junto + PIP por IK 1-GDL siguiendo la punta ---
    seg_k    = [2 3 4];    % indices de finger_geo: medio, anular, menique
    mcp_slot = [4 6 8];
    pip_slot = [5 7 9];
    for j = 1:3
        m  = mcp_slot(j);
        p  = pip_slot(j);
        kk = seg_k(j);

        % MCP: junto con el indice
        setpoints_deg(m) = theta_0(m) + s_mcp * (theta_grasp(m) - theta_0(m));

        % PIP: IK de 1 GDL para igualar el avance fraccional de la punta,
        setpoints_deg(p) = pip_follow_tip(kk, setpoints_deg(m), ...
                                          theta_0(p), theta_grasp(p), f_pip);
    end

end


%% ============================================================
%  finger_geo
%% ============================================================
function [L0, L1, L2, L3, id] = finger_geo(k)
    G = [ 27.0  55.000  38.160  34.000;   % 1 indice
          27.0  69.000  45.500  33.020;   % 2 medio
          27.0  58.996  38.160  28.482;   % 3 anular
          27.0  45.510  33.210  22.000 ]; % 4 menique
    ids = {'indice','medio','anular','menique'};
    L0 = G(k,1);  L1 = G(k,2);  L2 = G(k,3);  L3 = G(k,4);
    id = ids{k};
end


%% ============================================================
%  pip_follow_tip
%% ============================================================
function pip = pip_follow_tip(k, mcp_deg, pip_min, pip_max, f)
    [L0, L1, L2, L3, id] = finger_geo(k);
    P_MCP = [L0; 0];

    f = min(1, max(0, f));

    tip_a = forwardFinger(mcp_deg, pip_min, L1,L2,L3,P_MCP,id);
    tip_b = forwardFinger(mcp_deg, pip_max, L1,L2,L3,P_MCP,id);

    span = norm(tip_b - tip_a);
    if span < 1e-6
        % Degenerado: cae a interpolacion lineal del angulo
        pip = pip_min + f*(pip_max - pip_min);
        return;
    end
    d_target = f * span;

    % Biseccion sobre el PIP (distancia de punta monotona creciente)
    lo = pip_min;  hi = pip_max;
    for it = 1:40
        mid   = 0.5*(lo + hi);
        tip_m = forwardFinger(mcp_deg, mid, L1,L2,L3,P_MCP,id);
        if norm(tip_m - tip_a) < d_target
            lo = mid;
        else
            hi = mid;
        end
    end
    pip = 0.5*(lo + hi);
    pip = max(pip_min, min(pip_max, pip));   % tope duro de seguridad
end


%% ============================================================
%  calcular_cilindro_target
%% ============================================================
function theta_grasp = calcular_cilindro_target(theta_0, cyl_yc, cyl_zc, cyl_R, abd_pulgar_obj, mcp_pulgar_obj)
    theta_grasp = theta_0;

    % ---- Geometria por dedo: [L0 L1 L2 L3]  (de tu IK) ----
    %        indice / medio / anular / menique
    geo = [ 27.0  55.000  38.160  34.000;   % indice
            27.0  69.000  45.500  33.020;   % medio
            27.0  58.996  38.160  28.482;   % anular
            27.0  45.510  33.210  22.000 ]; % menique
    fid = {'indice','medio','anular','menique'};

    mcp_slot = [2 4 6 8];
    pip_slot = [3 5 7 9];

    for k = 1:4
        [th2, th3] = solve_tip_on_cylinder(geo(k,1), geo(k,2), geo(k,3), geo(k,4), ...
                                           cyl_yc, cyl_zc, cyl_R, fid{k});
        theta_grasp(mcp_slot(k)) = th2;   % MCP propio del dedo
        theta_grasp(pip_slot(k)) = th3;   % PIP propio del dedo
    end

    % ---- Pulgar: CIERRA sobre el cilindro, lo mas cerca posible del indice ----
    [th1_p, th2_p, ok_p] = solve_thumb_grip(cyl_yc, cyl_zc, cyl_R, 33.3, 60.0);
    if ok_p
        theta_grasp(10) = th2_p;   % MCP pulgar (= th2, flexion)
        theta_grasp(11) = th1_p;   % ABD pulgar (= th1, abduccion)
    else
        theta_grasp(10) = mcp_pulgar_obj;   % fallback hardcodeado
        theta_grasp(11) = abd_pulgar_obj;
    end

    % ABD_indice (slot 1) queda en reposo (= theta_0(1)).
end


%% ============================================================
function [th1_best, th2_best, ok] = solve_thumb_grip(yc,zc,R,X_max,ABD_min)

    if nargin<4, X_max   = 33.3; end
    if nargin<5, ABD_min = 60;   end

    %% Geometría
    L0 = 45.510;
    L1 = 33.210;
    L2 = 22.000;
    pos_base = [26.127;-73.939;0];
    tilt = 50;
    Yl = [cosd(tilt); sind(tilt);0];
    Xl = [sind(tilt);-cosd(tilt);0];
    Zl = cross(Xl,Yl);  Zl = Zl/norm(Zl);
    R_base = [Xl,Yl,Zl];
    C = [yc;zc];

    %% Inicialización
    ok = false;
    mejorCosto = inf;
    th1_best = 70;
    th2_best = 30;

    %% Barrido: por cada ABD (th1), buscar el PRIMER contacto en flexion (th2).
    for th1 = ABD_min:1:90

        dist_prev   = 0;
        have_prev   = false;
        th2_contact = 0;
        P_contact   = zeros(3,1);
        found       = false;

        for th2 = 0:1:90
            P    = fk_pulgar(th1,th2,L0,L1,L2,pos_base,R_base);
            dist = norm(P(2:3)-C);

            if have_prev && (dist_prev > R) && (dist <= R)
                frac        = (dist_prev - R) / (dist_prev - dist);
                th2_contact = (th2-1) + frac;
                P_contact   = fk_pulgar(th1,th2_contact,L0,L1,L2,pos_base,R_base);
                found       = true;
                break;
            end
            dist_prev = dist;
            have_prev = true;
        end

        if ~found
            fprintf('  th1=%2d : NO toca\n', int32(th1));
            continue;
        end

        viola_X = P_contact(1) > X_max;
        viola_Z = P_contact(3) < zc;
        fprintf('  th1=%2d : th2c=%5.1f  X=%7.2f  Y=%7.2f  Z=%7.2f  | viola_X=%g viola_Z=%g\n', ...
                int32(th1), th2_contact, P_contact(1), P_contact(2), P_contact(3), ...
                double(viola_X), double(viola_Z));

        if viola_X, continue; end
        if viola_Z, continue; end

        costo = 0.03*(X_max - P_contact(1));
        if costo < mejorCosto
            mejorCosto = costo;
            th1_best   = th1;
            th2_best   = th2_contact;
            ok         = true;
        end
    end
    if ok
        fprintf('[Pulgar] Contacto (primer cruce) encontrado.\n');
        fprintf('ABD = %.1f\n', th1_best);
        fprintf('MCP = %.1f\n', th2_best);
        fprintf('Costo = %.3f\n', mejorCosto);
    else
        fprintf('[Pulgar] No existe solucion valida. Se usa fallback.\n');
    end

end
%% ============================================================
%  thumb_ik_point
%% ============================================================
function [th1, th2, P] = thumb_ik_point(tgt3)
    L0 = 45.510;  L1 = 33.210;  L2 = 22.000;
    pos_base = [26.127; -73.939; 0];
    tilt = 50;
    Yl = [ cosd(tilt);  sind(tilt); 0];
    Xl = [ sind(tilt); -cosd(tilt); 0];
    Zl = cross(Xl, Yl);  Zl = Zl / norm(Zl);
    R_base = [Xl, Yl, Zl];

    lambda = 1.5;
    th1 = 40;  th2 = 40;

    for it = 1:600
        P = fk_pulgar(th1, th2, L0, L1, L2, pos_base, R_base);
        e = tgt3 - P;
        if norm(e) < 0.05, break; end
        h  = 1e-4;
        c1 = (fk_pulgar(th1+h,th2,L0,L1,L2,pos_base,R_base) ...
            - fk_pulgar(th1-h,th2,L0,L1,L2,pos_base,R_base)) / (2*h);
        c2 = (fk_pulgar(th1,th2+h,L0,L1,L2,pos_base,R_base) ...
            - fk_pulgar(th1,th2-h,L0,L1,L2,pos_base,R_base)) / (2*h);
        J  = [c1, c2];
        dq = (J'*J + lambda^2*eye(2)) \ (J'*e);
        th1 = max(0, min(90, th1 + 0.4*dq(1)));
        th2 = max(0, min(90, th2 + 0.4*dq(2)));
    end

    P = fk_pulgar(th1, th2, L0, L1, L2, pos_base, R_base);
end


%% ============================================================
%  fk_pulgar  (FK 3D del pulgar, identica a tu resolverPulgar)
%% ============================================================
function P = fk_pulgar(th1, th2, L0, L1, L2, pos_base, R_base)
    th3 = DIP_from_PIP(th2, 'pulgar');
    d_ext_loc = [0; 1; 0];
    d_flex0   = [0; 0; 1];
    d_abd_ax  = [1; 0; 0];

    d_abd     = rotAxis(d_ext_loc, d_abd_ax, th1);
    d_flex_ax = rotAxis(d_flex0,   d_abd_ax, th1);   % rota con th1

    P1   = L0 * d_abd;
    dir1 = rotAxis(d_abd, d_flex_ax, th2);   P2 = P1 + L1 * dir1;
    dir2 = rotAxis(dir1,  d_flex_ax, th3);   P3 = P2 + L2 * dir2;

    P = pos_base + R_base * P3;
end


%% ============================================================
%  rotAxis — Rodrigues (identica a tu IK). Devuelve vector unitario.
%% ============================================================
function v_rot = rotAxis(v, ax, ang_deg)
    ax = ax / norm(ax);
    K  = [     0,   -ax(3),  ax(2); ...
           ax(3),       0,  -ax(1); ...
          -ax(2),   ax(1),      0 ];
    Rm    = eye(3) + sind(ang_deg)*K + (1 - cosd(ang_deg))*(K*K);
    v_rot = Rm * v;
    v_rot = v_rot / norm(v_rot);
end


%% ============================================================
%  solve_tip_on_cylinder
%% ============================================================
function [th2, th3] = solve_tip_on_cylinder(L0, L1, L2, L3, yc, zc, R, dedo_id)

    P_MCP  = [L0;0];
    C      = [yc;zc];
    lambda = 0.01;

    % Inicialización
    th2 = 20;
    th3 = 30;

    %-------------------------------------------------------
    if R <= 30
        alpha = 1.0;
    elseif R >= 80
        alpha = 0.40;
    else
        alpha = 1.0 - (R-30)*(0.60)/(80-30);
    end

    for outer = 1:60

        [tip,joints] = forwardFinger(th2,th3,L1,L2,L3,P_MCP,dedo_id);

        % joints:

        P3 = joints(:,3);
        P4 = joints(:,4);

        % Punto de contacto
        contact = P3 + alpha*(P4-P3);

        v = contact - C;
        nv = norm(v);

        if nv < 1e-9
            nv = 1e-9;
        end

        tgt = C + R*v/nv;

        for it = 1:40

            [tip,joints] = forwardFinger(th2,th3,L1,L2,L3,P_MCP,dedo_id);

            P3 = joints(:,3);
            P4 = joints(:,4);

            contact = P3 + alpha*(P4-P3);

            e = tgt - contact;

            if norm(e) < 0.05
                break;
            end

            J_tip = jacobianFinger(th2,th3,L1,L2,L3,dedo_id);

            % Jacobiano aproximado del punto de contacto
            J = alpha*J_tip;

            dq = (J'*J + lambda^2*eye(2))\(J'*e);

            th2 = max(0,min(90,th2+dq(1)));
            th3 = max(0,min(90,th3+dq(2)));

        end

        [tip,joints] = forwardFinger(th2,th3,L1,L2,L3,P_MCP,dedo_id);

        P3 = joints(:,3);
        P4 = joints(:,4);

        contact = P3 + alpha*(P4-P3);

        if abs(norm(contact-C)-R) < 0.05
            break;
        end

    end

end


%% ============================================================
%  forwardFinger  (plano de flexion (r,z); identico a tu IK)
%% ============================================================
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
%  jacobianFinger  (identico a tu IK)
%% ============================================================
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
%  DIP_from_PIP  (mismos polinomios que tu IK)
%% ============================================================
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
%  dDIP_dPIP  (derivada, identica a tu IK)
%% ============================================================
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
        otherwise
            dth4 = 0;
    end
end


%% ============================================================
%  rot  (rotacion 2D)
%% ============================================================
function R = rot(th)
    R = [cosd(th) -sind(th);
         sind(th)  cosd(th)];
end