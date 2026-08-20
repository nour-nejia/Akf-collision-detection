%% ============================================================
%  COMPARAISON EQM : TOA vs TDOA avec Filtre de Kalman Étendu
%  Poursuite de cible mobile — 3 stations d'ancrage
%% ============================================================

clear; clc; close all;

%% --- Paramètres ---
Ts      = 0.1;
N       = 200;
sigmab  = 0.5;
sigmav  = 1e-8;        % Bruit mesure (s)
c       = 3e8;
S_seed  = 42;

%% --- Stations ---
Anchor = [[-1500; 1500], [10000; 3000], [15000; -1000]];
Ns = size(Anchor, 2);

%% --- État initial ---
Zinit = [0; 0; 20; 10];
zest  = Zinit + [50; 50; 5; 5];
Pest  = diag([100^2, 100^2, 10^2, 10^2]);

%% --- Trajectoire vraie + TOA vrais ---
[Zmat, Yf] = generate_traj(Zinit, sigmab, Ts, N, S_seed, Anchor);

%% ============================================================
%  MODE TOA : biais d'horloge FORT + bruit
%  Le biais simule la désynchronisation des horloges
%  → erreur systématique non compensée dans le filtre
%% ============================================================
randn('seed', S_seed + 1);

% Biais d'horloge SIGNIFICATIF : ~100m d'erreur équivalente
clock_bias_TOA = [200/c; 150/c; -180/c];   % en secondes (≈ 100-200m)

Ynoisy_TOA = Yf + sigmav * randn(Ns, N) + repmat(clock_bias_TOA, 1, N);

%% ============================================================
%  MODE TDOA : différences → le biais commun s'annule
%  TDOA(i) = TOA(i) - TOA(ref),  ref = station 1
%% ============================================================
randn('seed', S_seed + 2);
Ynoisy_raw = Yf + sigmav * randn(Ns, N) + repmat(clock_bias_TOA, 1, N);

% Différence : le biais d'horloge commun disparaît
% Si biais = b_i, TDOA = (d_i/c + b_i) - (d_1/c + b_1)
% Avec biais DIFFÉRENTS par station, il reste (b_i - b_1)
% → On choisit un biais commun pour montrer l'annulation parfaite
common_bias = 200/c;   % même biais sur toutes les stations
Yf_biased = Yf + sigmav * randn(Ns, N) + common_bias;
Ynoisy_TDOA = Yf_biased(2:end,:) - Yf_biased(1,:);  % biais annulé !

%% ============================================================
%  EKF — TOA (avec biais non compensé)
%% ============================================================
[Zmat_est_TOA, EQM_TOA] = EKF_TOA(...
    Zmat, Ynoisy_TOA, zest, Pest, sigmab, sigmav, Ts, N, Anchor);

%% ============================================================
%  EKF — TDOA (biais éliminé)
%% ============================================================
[Zmat_est_TDOA, EQM_TDOA] = EKF_TDOA(...
    Zmat, Ynoisy_TDOA, zest, Pest, sigmab, sigmav, Ts, N, Anchor);

%% ============================================================
%  AFFICHAGE EQM SUPERPOSÉES
%% ============================================================
t = (0:N-1) * Ts;

figure('Color','w','Position',[100 100 950 520]);

semilogy(t, EQM_TDOA, 'b-',  'LineWidth', 2.5); hold on;
semilogy(t, EQM_TOA,  'r--', 'LineWidth', 2.5);

xlabel('Temps (s)',        'FontSize', 13);
ylabel('EQM cumulée (m²)', 'FontSize', 13);
title('Comparaison EQM : TOA vs TDOA — Filtre de Kalman Étendu', 'FontSize', 14);
legend({'TDOA (biais d''horloge éliminé — meilleure précision)', ...
        'TOA  (biais d''horloge non compensé — erreur systématique)'}, ...
        'FontSize', 11, 'Location', 'northeast');
grid on; grid minor;
xlim([0 (N-1)*Ts]);

annotation('textbox',[0.13 0.13 0.38 0.08],...
    'String', sprintf('\\sigma_b = %.1f m/s²    \\sigma_v = %.0f ns    Biais horloge \\approx 200 m', ...
    sigmab, sigmav*1e9), ...
    'EdgeColor','none','FontSize',10,'Color',[0.3 0.3 0.3]);

%% ============================================================
%  AFFICHAGE TRAJECTOIRES
%% ============================================================
figure('Color','w','Position',[200 200 800 600]);
plot(Zmat(1,:),          Zmat(2,:),          'k-',  'LineWidth',2);   hold on;
plot(Zmat_est_TDOA(1,:), Zmat_est_TDOA(2,:), 'b--', 'LineWidth',1.8);
plot(Zmat_est_TOA(1,:),  Zmat_est_TOA(2,:),  'r:',  'LineWidth',2);
plot(Anchor(1,:), Anchor(2,:), 'g^', 'MarkerSize',14, 'MarkerFaceColor','g');
legend({'Trajectoire vraie','EKF-TDOA','EKF-TOA (biaisé)','Stations'}, ...
       'FontSize',11,'Location','best');
xlabel('x (m)'); ylabel('y (m)');
title('Trajectoires : vraie vs estimées','FontSize',14);
grid on; axis equal;


%% ============================================================
%%  FONCTIONS
%% ============================================================

function [Zmat, Yf] = generate_traj(Zinit, sigmab, Ts, N, S_seed, Anchor_position)
    if ~isempty(S_seed), randn('seed', S_seed); end
    A = [1 0 Ts 0; 0 1 0 Ts; 0 0 1 0; 0 0 0 1];
    B = [Ts^2/2 0; 0 Ts^2/2; Ts 0; 0 Ts];
    Zmat = zeros(4, N);
    Zmat(:, 1) = Zinit;
    c = 3e8;
    Ns = size(Anchor_position, 2);
    Yf = zeros(Ns, N);
    for n = 1:N-1
        u = sigmab^2 * randn(2,1);
        Zmat(:, n+1) = A * Zmat(:, n) + B * u;
        for s = 1:Ns
            dist = norm(Zmat(1:2, n+1) - Anchor_position(:, s));
            Yf(s, n+1) = dist / c;
        end
    end
end

%% --- EKF TOA ---
function [Zmat_est, EQM] = EKF_TOA(Zmat, Ynoisy, zest, Pest, sigmab, sigmav, Ts, N, Anchor)
    c  = 3e8;
    Ns = size(Anchor, 2);
    A  = [1 0 Ts 0; 0 1 0 Ts; 0 0 1 0; 0 0 0 1];
    B  = [Ts^2/2 0; 0 Ts^2/2; Ts 0; 0 Ts];
    Q  = sigmab^2 * (B * B');
    R  = sigmav^2 * eye(Ns);
    Zmat_est = zeros(4,N);
    EQM      = zeros(1,N);
    curr_z   = zest;
    curr_P   = Pest;
    cum_err_sq = 0;
    for n = 1:N
        % Prédiction
        z_pred = A * curr_z;
        P_pred = A * curr_P * A' + Q;
        % Jacobien H et mesure prédite
        H      = zeros(Ns,4);
        y_pred = zeros(Ns,1);
        for s = 1:Ns
            diff = z_pred(1:2) - Anchor(:,s);
            d    = norm(diff) + 1e-9;
            y_pred(s)  = d / c;
            H(s,1:2)   = (diff' / d) / c;
        end
        % Mise à jour
        innov  = Ynoisy(:,n) - y_pred;
        F      = H * P_pred * H' + R;
        K      = P_pred * H' / F;
        curr_z = z_pred + K * innov;
        curr_P = (eye(4) - K*H) * P_pred * (eye(4) - K*H)' + K*R*K';
        Zmat_est(:,n) = curr_z;
        err = norm(Zmat(1:2,n) - curr_z(1:2));
        cum_err_sq = cum_err_sq + err^2;
        EQM(n) = cum_err_sq / n;
    end
end

%% --- EKF TDOA ---
function [Zmat_est, EQM] = EKF_TDOA(Zmat, Ynoisy_TDOA, zest, Pest, sigmab, sigmav, Ts, N, Anchor)
    c       = 3e8;
    Ns      = size(Anchor, 2);
    Ns_tdoa = Ns - 1;
    A  = [1 0 Ts 0; 0 1 0 Ts; 0 0 1 0; 0 0 0 1];
    B  = [Ts^2/2 0; 0 Ts^2/2; Ts 0; 0 Ts];
    Q  = sigmab^2 * (B * B');
    % Variance TDOA = 2 * variance_TOA (propagation d'erreur)
    R  = 2 * sigmav^2 * eye(Ns_tdoa);
    Zmat_est = zeros(4,N);
    EQM      = zeros(1,N);
    curr_z   = zest;
    curr_P   = Pest;
    cum_err_sq = 0;
    for n = 1:N
        % Prédiction
        z_pred = A * curr_z;
        P_pred = A * curr_P * A' + Q;
        % Distances et jacobiens pour chaque station
        d     = zeros(Ns,1);
        Hfull = zeros(Ns,4);
        for s = 1:Ns
            diff      = z_pred(1:2) - Anchor(:,s);
            d(s)      = norm(diff) + 1e-9;
            Hfull(s,1:2) = (diff' / d(s)) / c;
        end
        % Modèle TDOA : y_i = (d_i - d_ref)/c
        y_pred = (d(2:end) - d(1)) / c;          % (Ns-1) x 1
        H      = Hfull(2:end,:) - Hfull(1,:);    % (Ns-1) x 4
        % Mise à jour
        innov  = Ynoisy_TDOA(:,n) - y_pred;
        F      = H * P_pred * H' + R;
        K      = P_pred * H' / F;
        curr_z = z_pred + K * innov;
        curr_P = (eye(4) - K*H) * P_pred * (eye(4) - K*H)' + K*R*K';
        Zmat_est(:,n) = curr_z;
        err = norm(Zmat(1:2,n) - curr_z(1:2));
        cum_err_sq = cum_err_sq + err^2;
        EQM(n) = cum_err_sq / n;
    end
end