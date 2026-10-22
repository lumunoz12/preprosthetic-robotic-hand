%% ============================================================
%  generar_cil_polinomios.m
%  ------------------------------------------------------------
%  Genera polinomios theta(R) POR JUNTA para el agarre cilindrico
%  y emite codigo C listo para pegar (cyl_targets.h / cyl_targets.c).
%
%  Idea: el cilindro queda FIJO en (CYL_YC, CYL_ZC); solo varia el
%  radio R. Para cada R se resuelve la postura objetivo con la MISMA
%  logica que tu simulacion (calcular_cilindro_target), que da un
%  angulo PROPIO por dedo (indice + medio + anular + menique + pulgar).
%  Luego se ajusta un polinomio de grado DEG a cada angulo.
%
%  En el STM32 solo evaluas los polinomios (Horner): coste ~cero,
%  no bloquea el lazo de control.
%
%  Requiere MATLAB R2016b+ (funciones locales dentro de un script).
%  Si tu MATLAB es mas viejo, avisame y lo parto en archivos.
% ============================================================
clear; clc; close all;

%% ---------------- CONFIG (AJUSTA A TU ESCENA) ----------------
R_MIN  = 15;      % [mm] radio minimo a cubrir
R_MAX  = 45;      % [mm] radio maximo a cubrir
R_STEP = 1;       % [mm] paso del barrido
DEG    = 3;       % grado del polinomio (3-4 suele bastar)

% Cilindro FIJO (mismos valores que tu main de simulacion)
CYL_YC = -35.0;   % [mm]
CYL_ZC =  50.0;   % [mm]

% Fallback del pulgar (igual que tu firmware) -- solo se usa si el
% cilindro queda fuera del alcance del pulgar para algun R.
ABD_PULGAR_OBJ = 75.0;
MCP_PULGAR_OBJ = 31.3;

OUT_H = 'cyl_targets.h';
OUT_C = 'cyl_targets.c';

%% ---------------- theta_0 (reposo, mano abierta) ----------------
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

%% ---------------- Barrido de R ----------------
Rs = (R_MIN:R_STEP:R_MAX).';
N  = numel(Rs);

% Columnas: i_mcp i_pip m_mcp m_pip a_mcp a_pip q_mcp q_pip p_mcp p_abd
%   (slots 2,3, 4,5, 6,7, 8,9, 10,11 de theta_grasp -> las 10 juntas
%    que tienen motor en el firmware)
TG       = zeros(N,10);
thumb_ok = false(N,1);

for i = 1:N
    [tg, ok] = calcular_cilindro_target(theta_0, CYL_YC, CYL_ZC, Rs(i), ...
                                        ABD_PULGAR_OBJ, MCP_PULGAR_OBJ);
    TG(i,:)  = [tg(2) tg(3)  tg(4) tg(5)  tg(6) tg(7)  tg(8) tg(9)  tg(10) tg(11)];
    thumb_ok(i) = ok;
end

%% ---------------- Chequeo de discontinuidad del pulgar ----------------
if any(~thumb_ok)
    warning(['El PULGAR usa FALLBACK en %d de %d radios. Eso mete un ', ...
             'salto en p_mcp/p_abd y el polinomio va a ajustar mal. ', ...
             'Restringe R al rango donde el pulgar SI alcanza, o trata ', ...
             'el pulgar aparte (LUT/piecewise).'], sum(~thumb_ok), N);
end

%% ---------------- Ajuste polinomial por junta ----------------
names  = {'i_mcp','i_pip','m_mcp','m_pip','a_mcp','a_pip','q_mcp','q_pip','p_mcp','p_abd'};
P      = cell(1,10);
maxerr = zeros(1,10);

for j = 1:10
    P{j}      = polyfit(Rs, TG(:,j), DEG);
    fitj      = polyval(P{j}, Rs);
    maxerr(j) = max(abs(fitj - TG(:,j)));
end

%% ---------------- Reporte de calidad de ajuste ----------------
fprintf('\n=== Error maximo de ajuste (grado %d, R=%.0f..%.0f mm) ===\n', ...
        DEG, R_MIN, R_MAX);
for j = 1:10
    flag = '';
    if maxerr(j) > 1.0, flag = '   <-- REVISAR (>1 deg)'; end
    fprintf('  %-6s : %6.3f deg%s\n', names{j}, maxerr(j), flag);
end
fprintf(['\nSi algun error es alto (tipicamente el pulgar), sube DEG, ', ...
         'restringe el rango de R, o usa LUT solo para esa junta.\n']);

%% ---------------- Graficos (dato vs ajuste) ----------------
figure('Name','theta(R) por junta','Color','w');
for j = 1:10
    subplot(2,5,j);
    plot(Rs, TG(:,j), 'b.', 'MarkerSize', 8); hold on;
    plot(Rs, polyval(P{j}, Rs), 'r-', 'LineWidth', 1.2);
    title(names{j}, 'Interpreter','none');
    xlabel('R [mm]'); ylabel('deg'); grid on;
end
legend({'IK','polyfit'}, 'Location','best');

%% ---------------- Verificacion puntual en un R ----------------
R_check = 38.0;   % <-- el radio que quieres inspeccionar

[tg_ik, ok_ik] = calcular_cilindro_target(theta_0, CYL_YC, CYL_ZC, R_check, ...
                                          ABD_PULGAR_OBJ, MCP_PULGAR_OBJ);
ik = [tg_ik(2) tg_ik(3) tg_ik(4) tg_ik(5) tg_ik(6) tg_ik(7) ...
      tg_ik(8) tg_ik(9) tg_ik(10) tg_ik(11)];

fprintf('\n=== Verificacion en R = %.1f mm  (thumb_ok=%d) ===\n', R_check, ok_ik);
fprintf('  %-6s  %8s  %8s  %7s\n', 'junta', 'IK', 'polyfit', 'dif');
for j = 1:10
    val_poly = polyval(P{j}, R_check);
    fprintf('  %-6s  %8.3f  %8.3f  %7.3f\n', ...
            names{j}, ik(j), val_poly, val_poly - ik(j));
end

%% ---------------- Emitir codigo C ----------------
emit_c(OUT_H, OUT_C, names, P, R_MIN, R_MAX);
fprintf('\nGenerado: %s  y  %s\n', OUT_H, OUT_C);
fprintf('Pegalos en tu proyecto y en main.c pon:  #include "cyl_targets.h"\n');


%% ============================================================
%%  EMISOR DE C
%% ============================================================
function emit_c(fh, fc, names, P, Rmin, Rmax)
    % ---- Header ----
    fid = fopen(fh, 'w');
    fprintf(fid, '#ifndef CYL_TARGETS_H\n#define CYL_TARGETS_H\n\n');
    fprintf(fid, '/* Auto-generado por generar_cil_polinomios.m */\n\n');
    fprintf(fid, '#define CYL_R_MIN  %.4ff\n',  Rmin);
    fprintf(fid, '#define CYL_R_MAX  %.4ff\n\n', Rmax);
    fprintf(fid, 'typedef struct {\n');
    fprintf(fid, '    float i_mcp, i_pip;\n');
    fprintf(fid, '    float m_mcp, m_pip;\n');
    fprintf(fid, '    float a_mcp, a_pip;\n');
    fprintf(fid, '    float q_mcp, q_pip;\n');
    fprintf(fid, '    float p_mcp, p_abd;\n');
    fprintf(fid, '} CylTargets;\n\n');
    fprintf(fid, 'void cyl_compute_targets(float R, CylTargets *o);\n\n');
    fprintf(fid, '#endif\n');
    fclose(fid);

    % ---- Source ----
    fid = fopen(fc, 'w');
    fprintf(fid, '#include "cyl_targets.h"\n\n');
    fprintf(fid, 'static float cil_poly(const float *c, int n, float x){\n');
    fprintf(fid, '    float acc = c[0];\n');
    fprintf(fid, '    int i; for(i=1;i<n;i++) acc = acc*x + c[i];\n');
    fprintf(fid, '    return acc;\n}\n\n');
    fprintf(fid, 'static float cil_clamp(float x){\n');
    fprintf(fid, '    if(x<0.0f) return 0.0f; if(x>90.0f) return 90.0f; return x;\n}\n\n');
    fprintf(fid, 'void cyl_compute_targets(float R, CylTargets *o){\n');
    fprintf(fid, '    if (R < CYL_R_MIN) R = CYL_R_MIN;\n');
    fprintf(fid, '    if (R > CYL_R_MAX) R = CYL_R_MAX;\n\n');

    for j = 1:numel(names)
        c = P{j};
        fprintf(fid, '    static const float c_%s[] = {', names{j});
        fprintf(fid, ' %.10gf', c(1));
        for k = 2:numel(c)
            fprintf(fid, ', %.10gf', c(k));
        end
        fprintf(fid, ' };\n');
    end
    fprintf(fid, '\n');
    for j = 1:numel(names)
        n = numel(P{j});
        fprintf(fid, '    o->%-5s = cil_clamp(cil_poly(c_%s, %d, R));\n', ...
                names{j}, names{j}, n);
    end
    fprintf(fid, '}\n');
    fclose(fid);
end


%% ============================================================
%%  NUCLEO DE CALCULO  (copiado de tu CylindricalGrasp_setpoint.m)
%%  - calcular_cilindro_target devuelve ademas thumb_ok
%%  - a solve_thumb_grip se le quitaron los fprintf (no cambia la
%%    matematica, solo evita spamear la consola en el barrido)
%% ============================================================
function [theta_grasp, thumb_ok] = calcular_cilindro_target(theta_0, cyl_yc, cyl_zc, cyl_R, abd_pulgar_obj, mcp_pulgar_obj)
    theta_grasp = theta_0;

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
        theta_grasp(mcp_slot(k)) = th2;
        theta_grasp(pip_slot(k)) = th3;
    end

    [th1_p, th2_p, ok_p] = solve_thumb_grip(cyl_yc, cyl_zc, cyl_R, 33.3, 60.0);
    thumb_ok = ok_p;
    if ok_p
        theta_grasp(10) = th2_p;   % MCP pulgar
        theta_grasp(11) = th1_p;   % ABD pulgar
    else
        theta_grasp(10) = mcp_pulgar_obj;   % fallback
        theta_grasp(11) = abd_pulgar_obj;
    end
end


function [th1_best, th2_best, ok] = solve_thumb_grip(yc,zc,R,X_max,ABD_min)
    if nargin<4, X_max   = 33.3; end
    if nargin<5, ABD_min = 60;   end

    L0 = 45.510;  L1 = 33.210;  L2 = 22.000;
    pos_base = [26.127;-73.939;0];
    tilt = 50;
    Yl = [cosd(tilt); sind(tilt);0];
    Xl = [sind(tilt);-cosd(tilt);0];
    Zl = cross(Xl,Yl);  Zl = Zl/norm(Zl);
    R_base = [Xl,Yl,Zl];
    C = [yc;zc];

    ok = false;
    mejorCosto = inf;
    th1_best = 70;
    th2_best = 30;

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
            continue;
        end

        viola_X = P_contact(1) > X_max;
        viola_Z = P_contact(3) < zc;
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
end


function P = fk_pulgar(th1, th2, L0, L1, L2, pos_base, R_base)
    th3 = DIP_from_PIP(th2, 'pulgar');
    d_ext_loc = [0; 1; 0];
    d_flex0   = [0; 0; 1];
    d_abd_ax  = [1; 0; 0];

    d_abd     = rotAxis(d_ext_loc, d_abd_ax, th1);
    d_flex_ax = rotAxis(d_flex0,   d_abd_ax, th1);

    P1   = L0 * d_abd;
    dir1 = rotAxis(d_abd, d_flex_ax, th2);   P2 = P1 + L1 * dir1;
    dir2 = rotAxis(dir1,  d_flex_ax, th3);   P3 = P2 + L2 * dir2;

    P = pos_base + R_base * P3;
end


function v_rot = rotAxis(v, ax, ang_deg)
    ax = ax / norm(ax);
    K  = [     0,   -ax(3),  ax(2); ...
           ax(3),       0,  -ax(1); ...
          -ax(2),   ax(1),      0 ];
    Rm    = eye(3) + sind(ang_deg)*K + (1 - cosd(ang_deg))*(K*K);
    v_rot = Rm * v;
    v_rot = v_rot / norm(v_rot);
end


function [th2, th3] = solve_tip_on_cylinder(L0, L1, L2, L3, yc, zc, R, dedo_id)
    P_MCP  = [L0;0];
    C      = [yc;zc];
    lambda = 0.01;

    th2 = 20;
    th3 = 30;

    if R <= 30
        alpha = 1.0;
    elseif R >= 80
        alpha = 0.40;
    else
        alpha = 1.0 - (R-30)*(0.60)/(80-30);
    end

    for outer = 1:60
        [tip,joints] = forwardFinger(th2,th3,L1,L2,L3,P_MCP,dedo_id); %#ok<ASGLU>
        P3 = joints(:,3);
        P4 = joints(:,4);
        contact = P3 + alpha*(P4-P3);
        v  = contact - C;
        nv = norm(v);
        if nv < 1e-9, nv = 1e-9; end
        tgt = C + R*v/nv;

        for it = 1:40
            [tip,joints] = forwardFinger(th2,th3,L1,L2,L3,P_MCP,dedo_id); %#ok<ASGLU>
            P3 = joints(:,3);
            P4 = joints(:,4);
            contact = P3 + alpha*(P4-P3);
            e = tgt - contact;
            if norm(e) < 0.05, break; end
            J_tip = jacobianFinger(th2,th3,L1,L2,L3,dedo_id);
            J  = alpha*J_tip;
            dq = (J'*J + lambda^2*eye(2))\(J'*e);
            th2 = max(0,min(90,th2+dq(1)));
            th3 = max(0,min(90,th3+dq(2)));
        end

        [tip,joints] = forwardFinger(th2,th3,L1,L2,L3,P_MCP,dedo_id); %#ok<ASGLU>
        P3 = joints(:,3);
        P4 = joints(:,4);
        contact = P3 + alpha*(P4-P3);
        if abs(norm(contact-C)-R) < 0.05, break; end
    end
end


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


function J = jacobianFinger(th2, th3, L1, L2, L3, dedo_id)
    k         = pi/180;
    th4       = DIP_from_PIP(th3, dedo_id); %#ok<NASGU>
    dth4_dth3 = dDIP_dPIP(th3, dedo_id);
    phi2r = deg2rad(th2);
    phi3r = deg2rad(th2 + th3);
    phi4r = deg2rad(th2 + th3 + DIP_from_PIP(th3, dedo_id));
    dphi4_dth2 = 1;
    dphi4_dth3 = 1 + dth4_dth3;
    dr_dth2 = k*(-sin(phi2r)*L1 - sin(phi3r)*L2 - sin(phi4r)*dphi4_dth2*L3);
    dz_dth2 = k*( cos(phi2r)*L1 + cos(phi3r)*L2 + cos(phi4r)*dphi4_dth2*L3);
    dr_dth3 = k*(-sin(phi3r)*L2 - sin(phi4r)*dphi4_dth3*L3);
    dz_dth3 = k*( cos(phi3r)*L2 + cos(phi4r)*dphi4_dth3*L3);
    J = [dr_dth2  dr_dth3;
         dz_dth2  dz_dth3];
end


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


function R = rot(th)
    R = [cosd(th) -sind(th);
         sind(th)  cosd(th)];
end