function weights = measurement_model_likelihood_field(map, particles, ranges, distance_map)
    n_particles = size(particles, 1);
    log_weights = zeros(n_particles, 1);
    
    alpha = linspace(-pi / 2, pi / 2, size(ranges, 1))';
    sigma = 0.2;  

    idx_valid = ~isnan(ranges);
    valid_ranges = ranges(idx_valid); 
    valid_alpha = alpha(idx_valid);  
  
    for i = 1:n_particles
        x_ray = particles(i, 1) + valid_ranges .* cos(particles(i, 3) + valid_alpha);
        y_ray = particles(i, 2) + valid_ranges .* sin(particles(i, 3) + valid_alpha);

        in_bounds = x_ray >= map.XWorldLimits(1) & x_ray <= map.XWorldLimits(2) & y_ray >= map.YWorldLimits(1) & y_ray <= map.YWorldLimits(2);
        x_ray = x_ray(in_bounds);
        y_ray = y_ray(in_bounds);

        % occupancy_values = getOccupancy(map, [x_ray(:) y_ray(:)]);
        % invalid_points = occupancy_values > 0.1 & occupancy_values < 0.6; % Solo zonas libres y poco ocupadas
        % x_ray = x_ray(~invalid_points);
        % y_ray = y_ray(~invalid_points);
        % if isempty(x_ray) || isempty(y_ray) || size(x_ray, 1) ~= size(y_ray, 1)
        %     disp('Ray invalid');
        %     continue;
        % end

        fila = round(y_ray * map.Resolution - map.YWorldLimits(1) * map.Resolution + 1);
        col = round(x_ray * map.Resolution + 1);

        % matrix_idx = world2grid(map, [x_ray(:) y_ray(:)]);
        % fila = matrix_idx(:, 1);
        % col = matrix_idx(:, 2);

        valid = fila >= 1 & fila <= size(distance_map, 1) & col >= 1 & col <= size(distance_map, 2);
        fila = fila(valid);
        col = col(valid);
        
        % Indexar de forma vectorial
        idx = sub2ind(size(distance_map), fila, col);
        dist = distance_map(idx);

        if numel(dist) >= 0.1 * numel(valid_ranges)
            log_prob = -0.5 * sum((dist .^ 2) / (sigma^2)); 
        else
            log_prob = -Inf; % Descartar si tiene pocas observaciones válidas
        end

        if getOccupancy(map, particles(i, 1:2)) > 0.1 
            log_prob = -Inf; % Penalizar si la partícula está en una zona ocupada
        end
        log_weights(i) = log_prob;

        % figure(550); 
        % show(map); hold on;
        % plot(x_ray, y_ray, 'r-', 'MarkerSize', 4);
        % plot(particles(i,1), particles(i,2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
        % plot(valid_ranges .* cos(valid_alpha), ...
        %      valid_ranges .* sin(valid_alpha), 'g.', 'MarkerSize', 4);
        % axis equal; 
        % title('Partículas y puntos de impacto LIDAR');
        % legend('Mapa', 'Puntos LIDAR', 'Partículas');
    end

    weights = exp(log_weights - max(log_weights));

    weight_sum = sum(weights);
    if weight_sum == 0 || isnan(weight_sum)
        weights = ones(n_particles, 1) / n_particles;
    else
        weights = weights / weight_sum;
    end
end
