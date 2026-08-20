clc; clear; close all;
N = 500;Nw = 20;Ns = 3;sigmav = 1e-6;  
P1 = [-1500;  1500];P2 = [10000;  3000];P3 = [15000; -1000];Anchor_pos = [P1 P2 P3]; 
Zinit = [1000; 1000; 0; 0];Ts = 1;eps_val = 100;     epsilon = 100;     zest = [0;0;0;0];Pest = epsilon * eye(4);
sigmab_values = [0.0001,0.001,0.01,sqrt(2),5,10,100,1000];
results = zeros(length(sigmab_values),3);
for i = 1:length(sigmab_values)
    sigmab = sigmab_values(i);
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
    results(i,:) = [sigmab, EQM_final, DtoP_final];

end
disp(' ');
disp('====== EFFET DE L ACCELERATION (sigmab) ======');
fprintf('%12s %15s %15s\n','sigmab','EQM_final','DtoP_final');

for i = 1:size(results,1)

    fprintf('%12.4f %15.6f %15.6f\n', ...
        results(i,1), ...
        results(i,2), ...
        results(i,3));
end