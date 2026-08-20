Anchor_pos = [-1500, 20000, 25000; 1500, 20000, 500]; 
Zinit = [1000; 1000; 0; 0];
sigmau = sqrt (2); 
sigmav = 1e-7; 
Ts = 1; 
N = 500;

[Zmat, Yf] = generate_traj(Zinit, sigmau, Ts, N, 50, Anchor_pos);

Ynoisy = Yf + sigmav * randn(size(Yf));

zest_init = [0; 0; 0; 0]; 
Pest_init = 0.01 * eye(4);
[Zest, EQM, DtoP, nP] = EKF_track_traj(Zmat, Ynoisy, zest_init, Pest_init, sigmau, sigmav, Ts, N,Anchor_pos);

figure;
plot(Zmat(1,:), Zmat(2,:), 'b', 'DisplayName', 'Vraie Trajectoire'); hold on;
plot(Zest(1,:), Zest(2,:), 'r--', 'DisplayName', 'Estimation EKF');
legend; title('Poursuite de cible par EKF');