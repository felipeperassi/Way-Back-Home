function new_particles = resample(particles, weights)
    new_particles = zeros(size(particles));
    M = size(particles, 1);
    dim = size(particles, 2);

    % Definir sigma fijo (por ejemplo, para x e y en metros, y theta en radianes)
    sigma = [0.167, 0.167, deg2rad(5)];  % suponiendo partículas [x, y, theta]

    cumulative_sum = cumsum(weights);
    cumulative_sum(end) = 1.0;

    step = 1 / M;
    start = rand * step;
    pointers = start + (0:M-1)' * step;

    i = 1;
    for m = 1:M
        while pointers(m) > cumulative_sum(i)
            i = i + 1;
        end
        noise = randn(1, dim) .* sigma;
        new_particles(m, :) = particles(i, :) + noise;
    end
end
    % M = size(particles, 1);
    % N_best = round(M * 0.6);
    % N_worst = M - N_best;

    % Neff = 1 / sum(weights.^2);
    % sigma_local = [3, 3, deg2rad(5)]; % Ruido agregado 
    
    % new_particles = sus(particles, weights, M);

    % [~, idx_sorted] = sort(weights, 'descend');
    % best_particles = particles(idx_sorted(1:N_best), :); % Seleccionar las mejores N_best partículas

    % corrected_particles = zeros(N_worst, 3);
    % for i = 1:N_worst
    %     idx = randi(N_best); 
    %     corrected_particles(i, :) = best_particles(idx, :) + randn(1, 3) .* sigma_local;
    %     if getOccupancy(map, corrected_particles(i, 1:2)) > 0.1
    %         corrected_particles(i, :) = best_particles(idx, :); 
    %     end
    % end
    % if Neff < 0.5 * M 
    %     new_particles = sus(particles, weights, M);

    %     [~, idx_sorted] = sort(weights, 'descend');
    %     best_particles = particles(idx_sorted(1:N_best), :); % Seleccionar las mejores N_best partículas

    %     corrected_particles = zeros(N_worst, 3);
    %     for i = 1:N_worst
    %         idx = randi(N_best); 
    %         corrected_particles(i, :) = best_particles(idx, :) + randn(1, 3) .* sigma_local;
    %         if getOccupancy(map, corrected_particles(i, 1:2)) > 0.1
    %             corrected_particles(i, :) = best_particles(idx, :); 
    %         end
    %     end
    % else
    %     new_particles = particles;
    % end

% n_local = round(0.3 * M); % 30% de las particulas
        % idx_local = randsample(1:M, n_local, true, weights);
        % local_particles = particles(idx_local, :) + randn(n_local, 3) .* sigma_local;

        % new_particles = [best_particles; local_particles];
        % M_new = size(new_particles, 1);  % nuevo número real de partículas
        % weights = ones(M_new, 1) / M_new;    % pesos iguales normalizados