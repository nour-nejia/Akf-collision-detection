

clc; clear; close all;

%% CHARGEMENT MODELE RF
MODEL_PATH = 'C:\Users\MSI\Desktop\Nour_Nejia\RT3\RT3_S2\Signaux_systèmes\TP\TP_KALMAN\Partie_D\classification_model.mat';
mdl = load(MODEL_PATH);

CL      = mdl.children_left;
CR      = mdl.children_right;
FT      = mdl.feature;
TH      = mdl.threshold;
VAL     = mdl.value;
N_TREES = mdl.n_trees;
CLASSES = mdl.classes;

disp('Modèle RF chargé.');
disp(CLASSES);

%% ===== SERIAL =====
COM_PORT  = 'COM4';
BAUD_RATE = 115200;

s = serialport(COM_PORT, BAUD_RATE);
configureTerminator(s, "LF");
flush(s);
disp('Serial port connecté.');

%% ===== PARAMETRES KALMAN =====
dt = 0.05;

A = [1, dt, 0.5*dt^2;
     0, 1,  dt;
     0, 0,  1];

sigma_jerk = 1.0;
Gamma      = [0.5*dt^2; dt; 1];
Q          = sigma_jerk^2 * (Gamma * Gamma');
sigma_d    = 0.015;
sigma_a    = 0.035;
R          = [sigma_d^2, 0; 0, sigma_a^2];
H          = [1 0 0; 0 0 1];

SEUIL_V_TTC = 0.05;

disp('En attente données...');
d_init = NaN;

while isnan(d_init)
    raw_line = strtrim(readline(s));
    parts    = str2double(split(raw_line, ','));
    if numel(parts) == 2 && all(isfinite(parts))
        d_init = parts(1);
    end
end

curr_z = [d_init; 0; 0];
curr_P = 100 * eye(3);

%% ===== VARIABLES =====
v_filt  = 0;
d_prev  = d_init;
rf_pred = "STATIONNAIRE";

%% ===== STOCKAGE PLOTS =====
Nmax       = 500;
d_raw_hist = nan(1, Nmax);
d_est_hist = nan(1, Nmax);
v_hist     = nan(1, Nmax);
a_hist     = nan(1, Nmax);
ttc_hist   = nan(1, Nmax);

%% ===== FIGURE =====
figure;

% Une seule box — prédiction RF
rfBox = annotation('textbox', [0.75 0.92 0.22 0.05], ...
    'String',              'RF : ---', ...
    'FontSize',            12, ...
    'FontWeight',          'bold', ...
    'HorizontalAlignment', 'center', ...
    'BackgroundColor',     [0.9 0.9 0.9]);

%% ===== BOUCLE PRINCIPALE =====
n = 0;

while true

    try
        raw_line = strtrim(readline(s));
    catch
        continue;
    end

    parts = str2double(split(raw_line, ','));
    if numel(parts) ~= 2 || ~all(isfinite(parts))
        continue;
    end

    y_d = parts(1);
    y_a = parts(2);

    %% — Filtre de Kalman —
    z_pred     = A * curr_z;
    P_pred     = A * curr_P * A' + Q;
    innovation = [y_d; y_a] - H * z_pred;
    S          = H * P_pred * H' + R;
    K          = P_pred * H' / S;
    curr_z     = z_pred + K * innovation;
    IKH        = eye(3) - K * H;
    curr_P     = IKH * P_pred * IKH' + K * R * K';

    d_est = curr_z(1);
    v_est = curr_z(2);
    a_est = curr_z(3);

    %% — Vitesse robuste —
    v_dist = (d_est - d_prev) / dt;
    d_prev = d_est;
    alpha_v = 0.7;
    v_filt  = alpha_v * v_filt + (1 - alpha_v) * v_est;
    v_used  = 0.5 * v_filt + 0.5 * v_dist;

    %% — TTC —
    if v_used < -SEUIL_V_TTC
        TTC = min(d_est / abs(v_used), 15);
    else
        TTC = 15;
    end

    %% — Stockage circulaire —
    n   = n + 1;
    idx = mod(n-1, Nmax) + 1;
    d_raw_hist(idx) = y_d;
    d_est_hist(idx) = d_est;
    v_hist(idx)     = v_used;
    a_hist(idx)     = a_est;
    ttc_hist(idx)   = TTC;

    %% — Prédiction RF (tous les 2 échantillons) —
    if mod(n, 2) == 0
        feat    = [d_est, v_est, a_est, v_used, TTC];
        rf_pred = rf_predict_mat(feat, CL, CR, FT, TH, VAL, N_TREES, CLASSES);
    end

    %% — Couleur selon classe RF —
    switch char(rf_pred)
        case 'APPROCHE',       color_rf = [0.4  0.9  0.4 ];
        case {'ELOIGNEMENT', 'ÉLOIGNEMENT'},    color_rf = [0.5  0.8  1.0 ];
        case 'COLLISION RISK', color_rf = [1.0  0.3  0.3 ];
        case 'FREINAGE',       color_rf = [1.0  0.8  0.0 ];
        case 'STATIONNAIRE' ,      color_rf = [0.85 0.85 0.85];
    end

    %% — Console + box RF —
    if mod(n, 10) == 0
        fprintf('%5d | d=%5.1f cm | v=%6.2f cm/s | TTC=%5.2f s | RF: %s\n', ...
            n, d_est*100, v_used*100, TTC, rf_pred);

        set(rfBox, 'String',          ['RF : ' char(rf_pred)], ...
                   'BackgroundColor', color_rf);
    end

    %% — Courbes (toutes les 10 trames) —
    if mod(n, 10) == 0
        t = 1:Nmax;

        subplot(4,1,1)
        plot(t, d_raw_hist*100, 'r--', 'LineWidth', 1.2); hold on;
        plot(t, d_est_hist*100, 'b',   'LineWidth', 2.0);
        hold off;
        title('Distance à l''obstacle'); ylabel('cm');
        legend('Brute','Filtrée KF','Location','best'); grid on;

        subplot(4,1,2)
        plot(t, v_hist*100, 'g', 'LineWidth', 2);
        yline(0, 'k--');
        title('Vitesse estimée'); ylabel('cm/s'); grid on;

        subplot(4,1,3)
        plot(t, a_hist, 'Color', [0.85 0.4 0], 'LineWidth', 2);
        yline(0, 'k--');
        title('Accélération filtrée KF'); ylabel('m/s²'); grid on;

        subplot(4,1,4)
        plot(t, ttc_hist, 'b', 'LineWidth', 2);
        yline(3, 'g--', 'warning');
        yline(1, 'r--', 'danger');
        title('Time To Collision'); ylabel('s'); xlabel('Échantillon'); grid on;

        drawnow;
    end

end

% =========================================================
% FONCTION RF_PREDICT_MAT
% =========================================================
function label = rf_predict_mat(feat, CL, CR, FT, TH, VAL, n_trees, classes)
    n_classes = size(VAL, 3);
    votes     = zeros(1, n_classes);

    for t = 1:n_trees
        node = 1;
        while true
            if CL(t, node) == -1
                votes = votes + squeeze(VAL(t, node, :))';
                break;
            end
            f = FT(t, node);
            if feat(f + 1) <= TH(t, node)
                node = CL(t, node) + 1;
            else
                node = CR(t, node) + 1;
            end
        end
    end

    [~, best] = max(votes);

    if iscell(classes)
        label = classes{best};
    else
        label = strtrim(classes(best, :));
    end
end