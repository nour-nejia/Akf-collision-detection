%Déclaration des paramètres
N = 500;
NS=3;
sigmav =sqrt(1e-12);
sigmab=sqrt(2);
N_w = 1;
EQM_mat = zeros(N_w, N);
DtoP_mat = zeros(N_w, N);
norm_P_est_mat = zeros(N_w, N);
C = 3 * 10^8;
z_est_init = [0; 0; 0; 0]; 
P_est_init = 0.01 * eye(4);
Ts = 1;
P1 = [-1500;  1500];P2 = [10000;  3000];P3 = [15000; -1000];
Anchor_pos = [P1 P2 P3]; 
Zinit = [1000; 1000; 0; 0];
S_seed = 50;
%Génération de la trajectoire réelle et des TOA non bruités 
[Zmat, Yf] = generate_traj(Zinit, sigmab, Ts, N, S_seed, Anchor_pos);
for nw=1:N_w
    Ynoisy = Yf + sigmav * randn(NS,N);
    [Zmat_est, EQM, DtoP, norm_P_est] = EKF_track_traj(Zmat, Ynoisy, z_est_init, P_est_init, sigmab, sigmav, Ts, N,Anchor_pos);
    EQM_mat(nw, :) = EQM;
    DtoP_mat(nw, :) = DtoP;
    norm_P_est_mat(nw, :) = norm_P_est;
end
figure(1);
plot(Zmat(1,:), Zmat(2,:), 'b', 'DisplayName', 'Vraie Trajectoire'); hold on;
plot(Zmat_est(1,:), Zmat_est(2,:), 'r--', 'DisplayName', 'Estimation EKF');
legend; title('Poursuite de cible par EKF');
figure(2);
plot(1:N, EQM_mat(1,:), 'b-', 'LineWidth', 1.5);
xlabel('Pas de temps'); ylabel('EQM (m²)');
title('EQM — \sigma_v = 10^{-12}');
grid on;
figure(3);
yyaxis left
plot(1:N, DtoP_mat(1,:), 'r-', 'LineWidth', 1.5);
ylabel('DtoP (m)');
yyaxis right
plot(1:N, norm_P_est_mat(1,:), 'b--', 'LineWidth', 1.5);
ylabel('trace(P_{est})');
xlabel('Pas de temps');
title('DtoP et norm\_P\_est — \sigma_v = 10^{-12}');
legend({'DtoP','norm\_P\_est'}, 'Location','best');
grid on;