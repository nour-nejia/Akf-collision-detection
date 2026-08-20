clc; clear; close all;
N = 500;Nw = 20;Ns = 3;
sigmav = sqrt(1e-12);sigmab = sqrt(2);
P1 = [-1500;  1500];P2 = [10000;  3000];P3 = [15000; -1000];Anchor_pos = [P1 P2 P3]; 
Zinit = [1000; 1000; 0; 0];
Ts = 1;S_seed = 50;epsilon = 100;   zest = [0;0;0;0];
Pest = epsilon * eye(4);
eps_values = [80000,20000, 1000, 500,300,100,50,10,1,0.01,0.00001];
results = zeros(length(eps_values),4);
for i = 1:length(eps_values)
    eps_val = eps_values(i);
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
    nc = N; % valeur par défaut = pas de convergence
    for k = 2:N
        if abs(EQM_moy(k) - EQM_moy(k-1)) < eps_val
            nc = k;
            break;
        end
    end
    EQM_final = mean(EQM_moy(nc:N));
    DtoP_final = mean(DtoP_moy(nc:N));
    results(i,:) = [eps_val, nc, EQM_final, DtoP_final];
end
disp(' ');
disp('---------RESULTATS ------------');
fprintf('%12s %10s %15s %15s\n','eps','nc','EQM_final','DtoP_final');
for i = 1:size(results,1)
    fprintf('%12.1e %10d %15.6f %15.6f\n', ...
        results(i,1), ...
        results(i,2), ...
        results(i,3), ...
        results(i,4));
end