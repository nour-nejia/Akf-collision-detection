N = 500;Nw = 20;NS=3;
epsilon_values = [0.01, 10, 1000, 1e5];
sigmav =sqrt(1e-12);sigmab=sqrt(2);
P1 = [-1500;  1500];P2 = [10000;  3000];P3 = [15000; -1000];
Anchor_pos = [P1 P2 P3]; Zinit = [1000; 1000; 0; 0];
colors = {'b', 'r', 'y', 'm'};
EQM_mat = zeros(Nw, N, length(epsilon_values));
[Zmat, Yf] = generate_traj(Zinit, sigmab, 1, N, 50, Anchor_pos);
figure('Color','w'); hold on;
legend_handles = [];
for e = 1:length(epsilon_values)   
    epsilon = epsilon_values(e);
    P_est_init = epsilon * eye(4);
    for nw = 1:Nw 
        Ynoisy = Yf + sigmav*randn(NS,N);
        [~, EQM, ~, ~] = EKF_track_traj(Zmat, Ynoisy, [0;0;0;0], P_est_init, sigmab, sigmav, 1, N, Anchor_pos);
        EQM_mat(nw,:,e) = EQM;
        %  courbes fines (réalisations)
        plot(1:N, EQM, 'Color', colors{e}, 'LineWidth', 0.5);   
    end    
    %  moyenne (courbe épaisse)
    EQM_mean = mean(EQM_mat(:,:,e), 1);  
    h = plot(1:N, EQM_mean, 'Color', colors{e}, 'LineWidth', 3);    
    legend_handles = [legend_handles h];    
end
grid on;
xlabel('n');
ylabel('EQM');
title('C.2 - EQM(n) pour N_w=20 et différentes valeurs de \epsilon');
legend(legend_handles, ...
    {'\epsilon = 0.01','\epsilon = 10','\epsilon = 1000','\epsilon = 100000'}, ...
    'Location','northeast');
axis([0 500 0 3e6]);
hold off;