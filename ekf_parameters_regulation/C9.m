clc; clear; close all;
N = 500;Nw = 20;Ns = 5;
sigmav = sqrt(1e-12);
sigmau = sqrt(2);
sigmab = sqrt(2);
P1 = [-1500;  1500];P2 = [10000;  3000];P3 = [15000; -1000];P4 = [-2000; 2500];P5 = [15000; -2000];Anchor_pos = [P1 P2 P3 P4 P5]; 
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
figure;
scatter(Anchor_pos(1,:), Anchor_pos(2,:), 100, 'b', 'filled');
hold on;
for i = 1:Ns
    text(Anchor_pos(1,i)+200, Anchor_pos(2,i)+200, ['S' num2str(i)]);
end

title('Position des stations');
xlabel('X');
ylabel('Y');
grid on;
axis equal;