function [Zmat, Yf] = generate_traj(Zinit, sigmab, Ts, N, S_seed, Anchor_position)
    if ~isempty(S_seed)
        randn('seed', S_seed);
    end

    A = [1 0 Ts 0; 0 1 0 Ts; 0 0 1 0; 0 0 0 1]; 
    B = [Ts^2/2 0; 0 Ts^2/2; Ts 0;0 Ts]; 
    
    Zmat = zeros(4, N);
    Zmat(:, 1) = Zinit;
    c = 3e8; % Vitesse de la lumière

    Ns = size(Anchor_position, 2);
    Yf = zeros(Ns, N);

    for n = 1:N-1
        u = sigmab^2 * randn(2,1); %sigmab c l'amplitude du bruit
        Zmat(:, n+1) = A * Zmat(:, n) + B * u;
        
        % Génération des TOA non bruités (Modèle d'observation)
        for s = 1:Ns
            dist = norm(Zmat(1:2, n+1) - Anchor_position(:, s));
            Yf(s, n+1) = dist / c;
        end
    end
end