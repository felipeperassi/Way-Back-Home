function weights = measurement_model(particles, lidar, ranges, map)
    n_particles = size(particles, 1);
    log_weights = zeros(n_particles, 1);
    sigma = 0.07;

    for i = 1:n_particles
        simulated_ranges = lidar(particles(i, :)');
        diff = simulated_ranges - ranges;
        
        valid = ~isnan(diff);
        if sum(valid) > 5
            log_prob = -0.5 * mean((diff(valid).^2) / (sigma^2)); 
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
