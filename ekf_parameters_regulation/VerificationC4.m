clc; clear; close all;
N = 500;
Nw = 20;
Ns = 3;
sigmav = sqrt(1e-12);
sigmab = sqrt(2);
P1 = [-1500;  1500];
P2 = [10000;  3000];
P3 = [15000; -1000];
Anchor_pos = [P1 P2 P3]; 
Zinit = [1000; 1000; 0; 0];
Ts = 1;
epsilon = 100;
zest = [0;0;0;0];
Pest = epsilon * eye(4);
EQM_sum = zeros(1, N);
for w = 1:Nw
    [Zmat, Yf] = generate_traj(Zinit, sigmab, Ts, N, [], Anchor_pos);
    Ynoisy = Yf + sigmav * randn(Ns, N);
    [~, EQM, ~, ~] = EKF_track_traj( ...
        Zmat, Ynoisy, zest, Pest, sigmab, sigmav, Ts, N, Anchor_pos);
    EQM_sum = EQM_sum + EQM;

end
EQM_moy = EQM_sum / Nw;
figure;
plot(1:N, EQM_moy, 'r', 'LineWidth', 2);
title('EQM moyenne (epsilon = 100)');
xlabel('Iterations');
ylabel('EQM');
grid on;