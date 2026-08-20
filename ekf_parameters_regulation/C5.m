clc; clear; close all;
N = 500;Nw = 20;Ns = 3;
sigmab = sqrt(2);P1 = [-1500;  1500];P2 = [10000;  3000];P3 = [15000; -1000];Anchor_pos = [P1 P2 P3]; 
Zinit = [1000; 1000; 0; 0];
Ts = 1;
eps_val = 100;     % seuil de convergence
epsilon = 100;     % initialisation de P
zest = [0;0;0;0];
Pest = epsilon * eye(4);
sigma_v_values = [1e-3, 1e-5, 1e-6, 1e-8];
results = zeros(length(sigma_v_values),3);
for i = 1:length(sigma_v_values)
    sigmav = sigma_v_values(i);
    EQM_sum = zeros(1,N);
    DtoP_sum = zeros(1,N);
    for w = 1:Nw
        [Zmat, Yf] = generate_traj(Zinit, sigmab, Ts, N, [], Anchor_pos);
        Ynoisy = Yf + sigmav * randn(Ns, N);
        [~, EQM, DtoP, ~] = EKF_track_traj( ...
            Zmat, Ynoisy, zest, Pest, sigmab, sigmav, Ts, N, Anchor_pos);
        EQM_sum = EQM_sum + EQM;
        DtoP_sum = DtoP_sum + DtoP;
    end
    EQM_moy = EQM_sum / Nw;
    DtoP_moy = DtoP_sum / Nw;
    nc = N;
    for k = 2:N
        if abs(EQM_moy(k) - EQM_moy(k-1)) < eps_val
            nc = k;
            break;
        end
    end
    EQM_final = mean(EQM_moy(nc:N));
    DtoP_final = mean(DtoP_moy(nc:N));
    results(i,:) = [sigmav^2, EQM_final, DtoP_final];
end
disp(' ');
disp('====== EFFET DU BRUIT  ======');
fprintf('%12s %15s %15s\n','sigmav^2','EQM_final','DtoP_final');
for i = 1:size(results,1)

    fprintf('%12.1e %15.6f %15.6f\n', ...
        results(i,1), ...
        results(i,2), ...
        results(i,3));
end