N = 500;Nw = 20;NS=3;
epsilon = 100;sigmav =sqrt(1e-12);sigmab=sqrt(2);
Ts = 1;
P1 = [-1500;  1500];P2 = [10000;  3000];P3 = [15000; -1000];Anchor_pos = [P1 P2 P3]; Zinit = [1000; 1000; 0; 0];
Zinit = [1000; 1000; 0; 0];
z_est_init = [0; 0; 0; 0];
P_est_init = epsilon * eye(4);
EQM_mat = zeros(Nw, N);
DtoP_mat = zeros(Nw, N);
norm_P_est_mat = zeros(Nw, N);
[Zmat, Yf] = generate_traj(Zinit, sigmab, Ts, N, 50, Anchor_pos);
for nw = 1:Nw
    Ynoisy = Yf + sigmav* randn(NS,N);
    [~, EQM, DtoP, norm_P_est] = EKF_track_traj(Zmat, Ynoisy, z_est_init, P_est_init, sigmab, sigmav, Ts, N, Anchor_pos);
    EQM_mat(nw, :) = EQM;
    DtoP_mat(nw, :) = DtoP;
    norm_P_est_mat(nw, :) = norm_P_est;
end
% 5. Affichage des résultats
figure('Color', 'w'); hold on;
% Affichage des Nw courbes individuelles (couleurs claires)
plot(EQM_mat', 'Color', [0.7, 0.7, 1], 'HandleVisibility', 'off'); % Bleu clair
plot(DtoP_mat', 'Color', [1, 0.7, 0.7], 'HandleVisibility', 'off'); % Rouge clair
plot(norm_P_est_mat', 'Color', [0.7, 1, 0.7], 'HandleVisibility', 'off'); % Vert clair
% Affichage des moyennes (couleurs foncées)
plot(mean(EQM_mat), 'b', 'LineWidth', 2.5, 'DisplayName', 'EQM Moyenne');
plot(mean(DtoP_mat), 'r', 'LineWidth', 2.5, 'DisplayName', 'DtoP Moyen');
plot(mean(norm_P_est_mat), 'g', 'LineWidth', 2.5, 'DisplayName', 'Norme P Moyenne');
% Mise en forme
xlabel('Temps (itérations n)');
ylabel('Performances');
title(['C.3. Effet moyen du bruit (Nw=20, \epsilon=' num2str(epsilon) ')']);
legend('show');
grid on;
