function [Zmat_est, EQM, DtoP, norm_P_est] = EKF_track_traj(...
    Zmat, Ynoisy, zest, Pest, sigmab, sigmav, Ts, N, Anchor)

A = [1 0 Ts 0; 0 1 0 Ts; 0 0 1 0; 0 0 0 1];
B = [Ts^2/2 0; 0 Ts^2/2; Ts 0; 0 Ts];

Ns = size(Anchor,2);

Q = sigmab^2 * (B * B');
R = sigmav^2 * eye(Ns);

Zmat_est = zeros(4,N);
EQM = zeros(1,N);
DtoP = zeros(1,N);
norm_P_est = zeros(1,N);

curr_z = zest;
curr_P = Pest;

cum_err_sq = 0;
cum_dist = 0;

for n = 1:N
    
    z_pred = A * curr_z;
    P_pred = A * curr_P * A' + Q;

    H = zeros(Ns,4);
    y_pred = zeros(Ns,1);

    for s = 1:Ns
        diff = z_pred(1:2) - Anchor(:,s);
        d = norm(diff) + 1e-9;

        y_pred(s) = d / (3e8); % TEMPS
        H(s,1:2) = (diff' / d) / (3e8);
    end
    F = H * P_pred * H' + R;
    K = P_pred * H' / F;
    curr_z = z_pred + K * (Ynoisy(:,n) - y_pred);
    curr_P = (eye(4) - K*H)*P_pred*(eye(4) - K*H)' + K*R*K';
    Zmat_est(:,n) = curr_z;
    err = norm(Zmat(1:2,n) - curr_z(1:2));
    cum_dist = cum_dist + err;
    cum_err_sq = cum_err_sq + err^2;
    DtoP(n) = cum_dist / n;
    EQM(n) = cum_err_sq / n;
    norm_P_est(n) = trace(curr_P);
end
end