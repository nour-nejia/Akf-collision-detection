N = 500;Nw = 20;NS = 5;epsilon = 100;biais_horloge=0.5e-5;sigmav = sqrt(1e-12);sigmab = sqrt(2);
P1 = [-1500; 1500];P2 = [10000; 3000];P3 = [15000; -1000];P4 = [-2000; 2500];P5 = [15000; -2000];Anchor_pos = [P1 P2 P3 P4 P5];
Zinit = [1000; 1000; 0; 0];colors = {'b'};EQM_mat = zeros(Nw, N);
[Zmat, Yf] = generate_traj(Zinit, sigmab, 1, N, 50, Anchor_pos);Yf_Tdoa=Yf;Zmat_tdoa=Zmat;legend_handles = [];P_est_init = epsilon * eye(4);
for nw = 1:Nw
    Ynoisy = Yf +biais_horloge+ sigmav * randn(NS, N);
    [~, EQM, ~, ~] = EKF_track_traj(Zmat, Ynoisy, [0;0;0;0], P_est_init, sigmab, sigmav, 1, N, Anchor_pos);
    EQM_mat(nw,:) = EQM;
end
EQM_mean = mean(EQM_mat, 1);
% TDOA 
EQM_mat_tdoa = zeros(Nw, N);
for nw = 1:Nw
    Y_tdoa = zeros(NS-1, N);
    Ynoisy = Yf_Tdoa +biais_horloge+ sigmav * randn(NS, N);
for i = 2:NS
    Y_tdoa(i-1,:) = Ynoisy(i,:) - Ynoisy(1,:);
end
[~, EQM_tdoa, ~, ~] = EKF_track_traj_TDOA(Zmat_tdoa, Y_tdoa, [0;0;0;0], P_est_init, sigmab, sigmav, 1, N, Anchor_pos);
EQM_mat_tdoa(nw,:) = EQM_tdoa;
end
EQM_mean_tdoa = mean(EQM_mat_tdoa, 1);
hold on;
% Tracer TOA
plot(1:N, EQM_mean, 'b', 'LineWidth', 3);
% Tracer TDOA
plot(1:N, EQM_mean_tdoa, 'r', 'LineWidth', 3);
grid on;xlabel('n');ylabel('EQM');title('Comparaison EQM moyen TOA vs TDOA - N_w=20');legend({'TOA', 'TDOA'}, 'Location', 'northeast');
axis([0 450 0 5e6]);
hold off;
