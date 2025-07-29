function weights = measurement_model_field(particles, lidar_angles, ranges, map, distance_field)
    n_particles = size(particles, 1);
    log_weights = zeros(n_particles, 1);
    sigma_hit = 0.07;

    for i = 1:n_particles
        pose = particles(i, :);  % [x, y, theta]
        x = pose(1);
        y = pose(2);
        theta = pose(3);

        % Proyectar rayos reales en el mapa desde esta pose
        n_rays = length(ranges);
        valid = ~isnan(ranges) & ~isinf(ranges);
        prob_sum = 0;
        count = 0;

        for j = 1:n_rays
            if ~valid(j)
                continue;
            end
            angle = theta + lidar_angles(j);
            range = ranges(j);

            % Posición esperada del obstáculo
            x_hit = x + range * cos(angle);
            y_hit = y + range * sin(angle);

            % Convertir a índices del mapa
            [ix, iy] = world2grid(map, [x_hit, y_hit]);

            % Verificar si está dentro del mapa
            if ix >= 1 && iy >= 1 && ix <= size(distance_field,1) && iy <= size(distance_field,2)
                dist = distance_field(ix, iy);
                prob = exp(-0.5 * (dist / sigma_hit)^2);  % gaussiana
                prob_sum = prob_sum + prob;
                count = count + 1;
            end
        end

        if count > 5
            log_weights(i) = log(prob_sum / count);
        else
            log_weights(i) = -Inf;
        end

        % Penalizar si está sobre obstáculo
        if getOccupancy(map, particles(i, 1:2)) > 0.1
            log_weights(i) = -Inf;
        end
    end

    % Normalizar
    weights = exp(log_weights - max(log_weights));
    weights = weights / sum(weights + eps);
end
