function weights = measurement_model_likelihood_field(map, particles, ranges, distance_map, resolution)
    n_particles = size(particles, 1);
    log_weights = zeros(n_particles, 1);
    alpha = linspace(-pi / 2, pi / 2, size(ranges, 1));
    sigma = 0.8;

    for i = 1:n_particles
        x_hit = particles(i, 1) + ranges .* cos(particles(i, 3) + alpha);
        y_hit = particles(i, 2) + ranges .* sin(particles(i, 3) + alpha);
        
        fila = round(y_hit / resolution);
        col = round(x_hit / resolution);
        
        valid = fila >= 1 & fila <= size(distance_map, 1) & ...
        col >= 1 & col <= size(distance_map, 2);

        fila = fila(valid);
        col = col(valid);
        % Indexar de forma vectorial
        idx = sub2ind(size(distance_map), fila, col);
        dist = distance_map(idx);
        disp('index:');
        disp(idx);

        % valid = ~isnan(dist);
        if numel(dist) > 5
            log_prob = -0.5 * mean(dist.^2) / (sigma^2); 
        else
            log_prob = -Inf; % Descartar si tiene pocas observaciones válidas
        end

        if getOccupancy(map, particles(i, 1:2)) > 0.1
            log_prob = -Inf; % Penalizar si la partícula está en una zona ocupada
        end
        log_weights(i) = log_prob;
    end

    weights = exp(log_weights - max(log_weights));

    weight_sum = sum(weights);
    if weight_sum == 0 || isnan(weight_sum)
        weights = ones(n_particles, 1) / n_particles;
    else
        weights = weights / weight_sum;
    end
end
