% =========================================================
% RF ADAPTATIF — PRÉDICTION sigma_jerk POUR FILTRE DE KALMAN
% Dataset : adapt_dataset_final.csv  (1600 lignes, 4 classes)
% Modèle  : Random Forest Régression (fitrensemble + Bag)
% Sortie  : mdl_adapt_Q_final.mat
% =========================================================

clc; clear; close all;

%% =========================================================
%% SECTION 1 : CHARGEMENT DONNÉES
%% =========================================================

fprintf('=== Chargement dataset ===\n');

DATA_PATH = 'adapt_dataset.csv';   % ← MODIFIÉ

if ~isfile(DATA_PATH)
    error('Fichier %s introuvable. Placez-le dans le dossier courant.', DATA_PATH);
end

T = readtable(DATA_PATH);
fprintf('  %d fenêtres chargées.\n', height(T));

% Features (7 colonnes) — statistiques sur fenêtre de 20 échantillons
X = [T.d_mean, T.d_std, T.a_abs, T.a_std, T.d_diff, T.v_abs, T.ttc_mean];
y = T.sigma_opt;   % cible : sigma_jerk optimal (0.1 / 1.0 / 2.5 / 5.0)

feat_names = {'d\_mean','d\_std','a\_abs','a\_std','d\_diff','v\_abs','ttc\_mean'};

fprintf('  Features : %d\n', size(X,2));
fprintf('  Distribution sigma_opt :\n');
sigma_vals = unique(y);
for i = 1:length(sigma_vals)
    n_i = sum(y == sigma_vals(i));
    fprintf('    sigma=%.1f → %d lignes (%.1f%%)\n', ...
            sigma_vals(i), n_i, 100*n_i/height(T));
end

%% =========================================================
%% SECTION 2 : SPLIT TRAIN / TEST (75% / 25%)
%% =========================================================

rng(42);
cv      = cvpartition(height(T), 'HoldOut', 0.25);
X_train = X(training(cv), :);
y_train = y(training(cv));
X_test  = X(test(cv),     :);
y_test  = y(test(cv));

fprintf('\nTrain : %d | Test : %d\n', length(y_train), length(y_test));

%% =========================================================
%% SECTION 3 : ENTRAÎNEMENT RF RÉGRESSION
%% =========================================================

fprintf('\n=== Entraînement Random Forest Régression ===\n');

mdl_adapt = fitrensemble(X_train, y_train, ...
    'Method',            'Bag',              ...
    'NumLearningCycles', 200,                ...   % ← augmenté (était 150)
    'Learners',          templateTree( ...
        'MaxNumSplits',      30,  ...              % ← augmenté (était 20)
        'MinLeafSize',        3), ...
    'ResponseName',      'sigma_jerk');

fprintf('  Entraînement terminé : 200 arbres.\n');

%% =========================================================
%% SECTION 4 : ÉVALUATION
%% =========================================================

y_pred_train = predict(mdl_adapt, X_train);
y_pred_test  = predict(mdl_adapt, X_test);

rmse_train = sqrt(mean((y_pred_train - y_train).^2));
rmse_test  = sqrt(mean((y_pred_test  - y_test ).^2));
mae_test   = mean(abs(y_pred_test - y_test));

ss_res = sum((y_test - y_pred_test).^2);
ss_tot = sum((y_test - mean(y_test)).^2);
r2     = 1 - ss_res / ss_tot;

fprintf('\n=== Résultats ===\n');
fprintf('  RMSE train : %.4f m/s³\n', rmse_train);
fprintf('  RMSE test  : %.4f m/s³\n', rmse_test);
fprintf('  MAE  test  : %.4f m/s³\n', mae_test);
fprintf('  R²   test  : %.4f\n',      r2);

%% =========================================================
%% SECTION 5 : IMPORTANCE DES FEATURES  ← CORRIGÉ
%% =========================================================

% oobPermutedPredictorImportance — méthode correcte pour fitrensemble Bag
imp = oobPermutedPredictorImportance(mdl_adapt);   % ← CORRIGÉ (était .oobPermutedVarDeltaError)

fprintf('\n=== Importance des features ===\n');
[imp_s, imp_i] = sort(imp, 'descend');
for i = 1:length(feat_names)
    fprintf('  %d. %-12s : %.6f\n', i, feat_names{imp_i(i)}, imp_s(i));
end

%% =========================================================
%% SECTION 6 : ERREUR OOB — CONVERGENCE
%% =========================================================
oob_err = mdl_adapt.oobLoss('Mode', 'cumulative');
fprintf('\nErreur OOB finale : %.4f\n', oob_err(end));

%% =========================================================
%% SECTION 7 : VISUALISATIONS
%% =========================================================

figure('Color','w','Position',[50,50,1300,850], ...
       'Name','RF Adaptatif — sigma_jerk Kalman');

sigma_colors = {[0.3,0.6,1.0],[0.3,0.85,0.3],[1.0,0.65,0.0],[1.0,0.2,0.2]};
% bleu=0.1  vert=1.0  orange=2.5  rouge=5.0

%% ---- 7.1 Prédiction vs Réel (test set) ----
subplot(3,3,1);
plot(y_test,      'b-',  'LineWidth', 1.5, 'DisplayName', 'σ réel');
hold on;
plot(y_pred_test, 'r--', 'LineWidth', 1.5, 'DisplayName', 'σ prédit RF');
legend('Location','best','FontSize',9);
ylabel('σ_{jerk} (m/s³)');
title(sprintf('Test set | RMSE=%.3f | R²=%.3f', rmse_test, r2));
grid on;

%% ---- 7.2 Scatter réel vs prédit ----
subplot(3,3,2);
for si = 1:length(sigma_vals)
    idx_s = (y_test == sigma_vals(si));
    scatter(y_test(idx_s), y_pred_test(idx_s), 35, ...
            sigma_colors{si}, 'filled', 'MarkerFaceAlpha', 0.6, ...
            'DisplayName', sprintf('σ=%.1f', sigma_vals(si)));
    hold on;
end
lims = [min(sigma_vals)-0.2, max(sigma_vals)+0.2];
plot(lims, lims, 'k--', 'LineWidth', 2, 'DisplayName', 'Idéal');
xlabel('σ réel'); ylabel('σ prédit');
title('Corrélation réel vs prédit');
legend('Location','best','FontSize',8);
grid on;

%% ---- 7.3 Distribution erreur ----
subplot(3,3,3);
err_hist = y_pred_test - y_test;
histogram(err_hist, 25, 'FaceColor', [0.2,0.5,0.8], 'FaceAlpha', 0.75);
xline(0, 'r--', 'LineWidth', 2);
xline(mean(err_hist), 'g--', 'LineWidth', 1.5, ...
      'Label', sprintf('μ=%.3f', mean(err_hist)));
xlabel('Erreur (prédit − réel)');
ylabel('Fréquence');
title(sprintf('Distribution erreur | MAE=%.3f', mae_test));
grid on;

%% ---- 7.4 Convergence OOB ----
%% ---- 7.4 Convergence OOB ----
subplot(3,3,4);
oob_err = mdl_adapt.oobLoss('Mode', 'cumulative');   % calcul ici directement
plot(1:length(oob_err), oob_err, 'b-', 'LineWidth', 2);
yline(oob_err(end), 'r--', 'LineWidth', 1.5, ...
      'Label', sprintf('Final=%.4f', oob_err(end)));
xlabel('Nombre d''arbres');
ylabel('Erreur OOB (MSE)');
title('Convergence du Random Forest');
grid on;
fprintf('\nErreur OOB finale : %.4f\n', oob_err(end));

%% ---- 7.5 Importance des features ----
subplot(3,3,5);
b = barh(imp_s, 'FaceColor', [0.2,0.5,0.8]);
yticks(1:length(feat_names));
yticklabels(feat_names(imp_i));
xlabel('Importance OOB');
title('Importance des Features');
for i = 1:length(imp_s)
    text(imp_s(i)+0.0002, i, sprintf('%.4f', imp_s(i)), ...
         'VerticalAlignment','middle','FontSize',8);
end
grid on;

%% ---- 7.6 Distribution sigma par label ----
subplot(3,3,6);
labels_unique = unique(T.label);
lbl_colors    = {[0.8,0.8,0.8],[0.4,0.9,0.4],[1.0,0.3,0.3],[0.5,0.8,1.0]};
hold on;
for li = 1:length(labels_unique)
    idx_l = strcmp(T.label, labels_unique{li});
    sig_l = T.sigma_opt(idx_l);
    scatter(li*ones(sum(idx_l),1), sig_l, 25, lbl_colors{li}, 'filled', ...
            'MarkerFaceAlpha', 0.45, 'DisplayName', labels_unique{li});
    plot(li, mean(sig_l), 'k^', 'MarkerSize', 9, 'MarkerFaceColor', 'k', ...
         'HandleVisibility','off');
end
xticks(1:length(labels_unique));
xticklabels(labels_unique);
xtickangle(20);
yticks(sigma_vals);
ylabel('σ_{jerk} optimal (m/s³)');
title('σ_{jerk} par état de mouvement');
legend('Location','best','FontSize',8);
grid on;

%% ---- 7.7 d_std vs sigma ----
subplot(3,3,7);
scatter(T.d_std*100, T.sigma_opt, 25, T.sigma_opt, 'filled', 'MarkerFaceAlpha',0.6);
colorbar; colormap('jet');
xlabel('d\_std (cm)');
ylabel('σ_{jerk}');
title('Variabilité distance → σ_{jerk}');
yticks(sigma_vals);
grid on;

%% ---- 7.8 v_abs vs sigma ----
subplot(3,3,8);
scatter(T.v_abs*100, T.sigma_opt, 25, T.sigma_opt, 'filled', 'MarkerFaceAlpha',0.6);
colorbar;
xlabel('|vitesse| moyenne (cm/s)');
ylabel('σ_{jerk}');
title('Vitesse → σ_{jerk}');
yticks(sigma_vals);
grid on;

%% ---- 7.9 TTC vs sigma ----
subplot(3,3,9);
scatter(T.ttc_mean, T.sigma_opt, 25, T.sigma_opt, 'filled', 'MarkerFaceAlpha',0.6);
colorbar;
xlabel('TTC moyen (s)');
ylabel('σ_{jerk}');
title('TTC → σ_{jerk}');
yticks(sigma_vals);
grid on;

sgtitle('RF Adaptatif — Prédiction σ_{jerk} Kalman — ESP32', ...
        'FontSize',13,'FontWeight','bold');

%% =========================================================
%% SECTION 8 : SAUVEGARDE MODÈLE
%% =========================================================

save('ML_adaptatif.mat', 'mdl_adapt');          % ← nom mis à jour
fprintf('\nModèle sauvegardé : ML_adaptatif.mat\n');

%% =========================================================
%% SECTION 9 : TEST INTÉGRATION TEMPS RÉEL
%% =========================================================

fprintf('\n=== Test prédiction temps réel ===\n');

% 4 exemples types — un par classe sigma
exemples = [
    0.18, 0.001, 0.010, 0.011, 0.001, 0.003, 15.0;   % calme stationnaire
    0.15, 0.010, 0.015, 0.018, 0.015, 0.025, 10.0;   % mouvement lent
    0.10, 0.025, 0.020, 0.030, 0.040, 0.060,  5.0;   % approche active
    0.04, 0.045, 0.035, 0.050, 0.080, 0.100,  1.5;   % collision imminente
];
sigma_attendus = [0.1, 1.0, 2.5, 5.0];
descs = {'Calme stationnaire', 'Mouvement lent', 'Approche active', 'Collision imminente'};

fprintf('  %-22s | σ_attendu | σ_prédit | Erreur\n', 'Scénario');
fprintf('  %s\n', repmat('-',1,60));
for i = 1:4
    sigma_p = predict(mdl_adapt, exemples(i,:));
    fprintf('  %-22s |    %.1f    |   %.3f  | %.3f\n', ...
            descs{i}, sigma_attendus(i), sigma_p, abs(sigma_p - sigma_attendus(i)));
end

fprintf('\n=== Code à insérer dans la boucle KF temps réel ===\n');
fprintf('\n%% Avant la boucle :\n');
fprintf('WIN_SIZE  = 20;\n');
fprintf('d_window  = d_init * ones(1, WIN_SIZE);\n');
fprintf('a_window  = zeros(1, WIN_SIZE);\n');
fprintf('v_window  = zeros(1, WIN_SIZE);\n');
fprintf('mdl_adapt = load(''mdl_adapt_Q_final.mat'').mdl_adapt;\n');
fprintf('\n%% Dans la boucle, après chaque mesure :\n');
fprintf('d_window = [d_window(2:end), y_d];\n');
fprintf('a_window = [a_window(2:end), y_a];\n');
fprintf('v_window = [v_window(2:end), abs(v_used)];\n');
fprintf('\n%% Mise à jour Q toutes les 10 trames :\n');
fprintf('if mod(n, 10) == 0\n');
fprintf('    feat_rt    = [mean(d_window), std(d_window), ...\n');
fprintf('                  mean(abs(a_window)), std(a_window), ...\n');
fprintf('                  max(abs(diff(d_window))), mean(v_window), TTC];\n');
fprintf('    sigma_new  = predict(mdl_adapt, feat_rt);\n');
fprintf('    sigma_jerk = 0.3*sigma_new + 0.7*sigma_jerk;  %% lissage exponentiel\n');
fprintf('    Q          = sigma_jerk^2 * (Gamma * Gamma'');\n');
fprintf('end\n');

fprintf('\nTerminé.\n');
