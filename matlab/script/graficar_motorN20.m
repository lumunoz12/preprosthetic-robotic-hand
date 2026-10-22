% =========================================================
%  graficar_motor.m
%  Analisis y graficado ENC N20 vs AS5600
%  Genera figuras exportables en SVG/PDF para informe
% =========================================================

clear; clc; close all;

%% ---- CARGAR DATOS ----
load('datos_motor.mat');   % carga variable 'datos'

targets  = datos.target;
enc_est  = datos.enc_est;
as_est   = datos.as_est;
error_deg = enc_est - as_est;

N = length(targets);

%% ---- ESTADISTICOS GLOBALES ----
rmse  = sqrt(mean(error_deg.^2));
mae   = mean(abs(error_deg));
bias  = mean(error_deg);
sigma = std(error_deg);

fprintf('===== Estadisticos de error ENC - AS5600 =====\n');
fprintf('  RMSE  : %.4f deg\n', rmse);
fprintf('  MAE   : %.4f deg\n', mae);
fprintf('  Bias  : %.4f deg\n', bias);
fprintf('  Sigma : %.4f deg\n', sigma);
fprintf('  Max   : %.4f deg\n', max(abs(error_deg)));
fprintf('==============================================\n');

%% =========================================================
%  FIGURA 1: Dispersion  ENC vs AS5600  (grafica identidad)
% =========================================================
fig1 = figure('Units','centimeters','Position',[2 2 14 12]);

% Linea de referencia ideal y=x
lim_min = min([targets, enc_est, as_est]) - 5;
lim_max = max([targets, enc_est, as_est]) + 5;
hold on;
plot([lim_min lim_max], [lim_min lim_max], '--', ...
     'Color',[0.6 0.6 0.6], 'LineWidth', 1.2, 'DisplayName','Ideal (y = x)');

% Puntos de dispersion
scatter(as_est, enc_est, 60, 'filled', ...
    'MarkerFaceColor',[0.12 0.47 0.71], ...
    'MarkerEdgeColor','k', 'LineWidth', 0.8, ...
    'DisplayName','Mediciones');

% Regresion lineal
p   = polyfit(as_est, enc_est, 1);
x_r = linspace(lim_min, lim_max, 100);
y_r = polyval(p, x_r);
plot(x_r, y_r, '-', 'Color',[0.84 0.15 0.16], 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Regresion: y=%.4fx%+.4f', p(1), p(2)));

% Anotacion de angulos
for i = 1:N
    text(as_est(i)+0.5, enc_est(i)+0.5, sprintf('%d°', targets(i)), ...
        'FontSize', 7, 'Color', [0.2 0.2 0.2]);
end

hold off;
xlim([lim_min lim_max]); ylim([lim_min lim_max]);
axis square;
xlabel('Ángulo AS5600 (°)', 'FontSize', 11);
ylabel('Ángulo Encoder N20 (°)', 'FontSize', 11);
title('Dispersión: Encoder N20 vs AS5600', 'FontSize', 12, 'FontWeight','bold');
legend('Location','northwest', 'FontSize', 9);
grid on; box on;

% Anotacion RMSE en la figura
annotation('textbox', [0.58 0.18 0.30 0.12], ...
    'String', {sprintf('RMSE = %.4f°', rmse), ...
                sprintf('MAE  = %.4f°', mae), ...
                sprintf('Bias = %.4f°', bias)}, ...
    'FontSize', 8, 'BackgroundColor','w', 'EdgeColor',[0.4 0.4 0.4]);

exportgraphics(fig1, 'fig1_dispersion.pdf', 'ContentType','vector');
exportgraphics(fig1, 'fig1_dispersion.svg', 'ContentType','vector');
fprintf('fig1_dispersion exportada.\n');

%% =========================================================
%  FIGURA 2: Error angular vs angulo objetivo
% =========================================================
fig2 = figure('Units','centimeters','Position',[18 2 14 10]);
hold on;

% Banda de tolerancia ±1°
fill([targets(1)-5, targets(end)+5, targets(end)+5, targets(1)-5], ...
     [1 1 -1 -1], [0.9 0.95 1.0], 'EdgeColor','none', ...
     'DisplayName','Tolerancia ±1°');

% Linea de error cero
yline(0, '--', 'Color',[0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off');

% Error por punto
plot(targets, error_deg, '-o', ...
    'Color',[0.12 0.47 0.71], 'LineWidth', 1.8, ...
    'MarkerFaceColor',[0.12 0.47 0.71], 'MarkerSize', 7, ...
    'DisplayName','Error (ENC - AS5600)');

% Banda sigma
yline( sigma, ':', 'Color',[0.84 0.15 0.16], 'LineWidth',1.2, ...
    'DisplayName',sprintf('+σ = %.4f°', sigma));
yline(-sigma, ':', 'Color',[0.84 0.15 0.16], 'LineWidth',1.2, ...
    'DisplayName',sprintf('-σ = %.4f°', sigma));

hold off;
xlim([targets(1)-5, targets(end)+5]);
xlabel('Ángulo objetivo (°)', 'FontSize', 11);
ylabel('Error angular (°)', 'FontSize', 11);
title('Error del Encoder N20 respecto al AS5600', 'FontSize', 12, 'FontWeight','bold');
legend('Location','best', 'FontSize', 9);
grid on; box on;

exportgraphics(fig2, 'fig2_error.pdf', 'ContentType','vector');
exportgraphics(fig2, 'fig2_error.svg', 'ContentType','vector');
fprintf('fig2_error exportada.\n');

%% =========================================================
%  FIGURA 3: Histograma de errores
% =========================================================
fig3 = figure('Units','centimeters','Position',[2 16 12 10]);

% Histograma
hh = histogram(error_deg, 'BinMethod','scott', ...
    'FaceColor',[0.12 0.47 0.71], 'EdgeColor','k', ...
    'FaceAlpha', 0.75, 'Normalization','probability');
hold on;

% Curva gaussiana ajustada
x_g = linspace(min(error_deg)-1, max(error_deg)+1, 200);
y_g = normpdf(x_g, bias, sigma);
% Escalar a probabilidad (bin width * normpdf)
bw  = hh.BinWidth;
plot(x_g, y_g * bw, '-', 'Color',[0.84 0.15 0.16], 'LineWidth', 2.0, ...
    'DisplayName', sprintf('Gaussiana μ=%.4f° σ=%.4f°', bias, sigma));

xline(bias, '--k', 'LineWidth', 1.2, 'DisplayName', sprintf('Media = %.4f°', bias));
hold off;

xlabel('Error (ENC - AS5600) [°]', 'FontSize', 11);
ylabel('Probabilidad', 'FontSize', 11);
title('Distribución del error angular', 'FontSize', 12, 'FontWeight','bold');
legend('FontSize', 9);
grid on; box on;

exportgraphics(fig3, 'fig3_histograma.pdf', 'ContentType','vector');
exportgraphics(fig3, 'fig3_histograma.svg', 'ContentType','vector');
fprintf('fig3_histograma exportada.\n');

%% =========================================================
%  FIGURA 4: Seguimiento temporal (todas las trayectorias)
% =========================================================
fig4 = figure('Units','centimeters','Position',[18 16 18 12]);

cmap = lines(N);
hold on;
legend_entries = {};

for i = 1:N
    t_vec  = datos.t_log{i};
    e_vec  = datos.enc_log{i};
    a_vec  = datos.as_log{i};

    if ~isempty(t_vec)
        % ENC solido, AS punteado, mismo color por angulo
        h1 = plot(t_vec, e_vec, '-',  'Color', cmap(i,:), 'LineWidth', 1.5);
        h2 = plot(t_vec, a_vec, '--', 'Color', cmap(i,:), 'LineWidth', 1.0);
        legend_entries{end+1} = sprintf('ENC %d°', targets(i)); %#ok<AGROW>
        legend_entries{end+1} = sprintf('AS  %d°', targets(i)); %#ok<AGROW>
    end
end

hold off;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('Ángulo (°)', 'FontSize', 11);
title('Seguimiento temporal: Encoder N20 (—) vs AS5600 (- -)', ...
    'FontSize', 12, 'FontWeight','bold');
legend(legend_entries, 'Location','eastoutside', 'FontSize', 7);
grid on; box on;

exportgraphics(fig4, 'fig4_temporal.pdf', 'ContentType','vector');
exportgraphics(fig4, 'fig4_temporal.svg', 'ContentType','vector');
fprintf('fig4_temporal exportada.\n');

%% ---- TABLA RESUMEN EN CONSOLA ----
fprintf('\n%-10s %-12s %-12s %-12s\n', 'Target(°)', 'ENC_est(°)', 'AS_est(°)', 'Error(°)');
fprintf('%s\n', repmat('-',1,50));
for i = 1:N
    fprintf('%-10d %-12.3f %-12.3f %-12.4f\n', ...
        targets(i), enc_est(i), as_est(i), error_deg(i));
end
fprintf('%s\n', repmat('-',1,50));
fprintf('%-10s %-12s %-12s %-12.4f\n', '', '', 'RMSE:', rmse);
fprintf('%-10s %-12s %-12s %-12.4f\n', '', '', 'MAE:',  mae);