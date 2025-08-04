function weights = measurement_model_likelihood_field(map, particles, ranges, angles, distance_map, occupancy_map)
    % MEASUREMENT_MODEL_LIKELIHOOD_FIELD - Calcula los pesos de las partículas utilizando el modelo de Likelihood Field.
    %
    % Entradas:
    %   map           - Mapa de ocupación (objeto occupancyMap)
    %   particles     - Matriz de partículas N x 3 con las poses de las partículas [x, y, theta]
    %   ranges        - Vector de distancias medidas por el LiDAR
    %   angles        - Vector de ángulos correspondientes a las mediciones LiDAR
    %   distance_map  - Mapa de distancias usado para la comparación de lecturas
    %   occupancy_map - Mapa de ocupación para la evaluación de obstáculos
    %
    % Salidas:
    %   weights       - Vector con los pesos normalizados, uno por cada partícula. Representa la probabilidad de que
    %                   cada partícula sea la posición real del robot, dado el mapa y las mediciones del LIDAR.

    % Número de partículas y número de rayos
    n_particles = size(particles, 1);
    n_ray = length(ranges);
    
    % Se filtran las mediciones del LiDAR para evitar NaN
    idx_valid_ranges = ~isnan(ranges);
    valid_ranges = ranges(idx_valid_ranges);
    valid_angles = angles(idx_valid_ranges);

    % Se calcula la posición de los rayos en el mundo para cada partícula
    x_ray = particles(:, 1) + cos(particles(:, 3) + valid_angles) .* valid_ranges';
    y_ray = particles(:, 2) + sin(particles(:, 3) + valid_angles) .* valid_ranges';

    % Se transforma las coordenadas del mundo a las del mapa
    dist_matrix = world2grid(map, [x_ray(:), y_ray(:)]);
    row = dist_matrix(:, 1);
    col = dist_matrix(:, 2);

    % Se filtran las coordenadas para evitar índices fuera de rango y NaN
    valid_rows = row >= 1 & row <= size(distance_map, 1);
    valid_cols = col >= 1 & col <= size(distance_map, 2);
    not_nan = ~isnan(row) & ~isnan(col);
    valid_mask = valid_rows & valid_cols & not_nan; 

    % Se filtran las posiciones válidas
    row = row(valid_mask);
    col = col(valid_mask);

    % Se obtiene el índice de las posiciones en el mapa de distancias
    idx_distance_map = sub2ind(size(distance_map), row, col);

    % Se inicializa el vector de distancias con infinito
    dist_vector = inf(n_particles, n_ray);

    % Se asignan las distancias del mapa de distancias a las posiciones válidas
    dist_vector(valid_mask) = distance_map(idx_distance_map);

    % Se define el mapa binario de zonas exploradas
    % Las zonas con ocupación entre [0.195, 0.65] se consideran no exploradas (grises)
    explored_map = (occupancy_map < 0.195) | (occupancy_map > 0.65);

    % Se verifica si cada celda donde impacta un rayo fue explorada
    explored_mask = false(n_particles, n_ray);
    explored_mask(valid_mask) = explored_map(idx_distance_map);

    % Si un rayo cae en una zona no explorada, se asigna NaN
    dist_vector(~explored_mask) = NaN;
    
    % Se define el desvío estándar del modelo gaussiano (ruido del sensor).
    sigma = 0.02;  

    % Se calcula la probabilidad de cada partícula usando el modelo gaussiano
    likelihoods = exp(-0.5 * (dist_vector / sigma).^2);
    likelihoods(isnan(likelihoods)) = 0; 

    % Se normalizan los pesos para que sumen 1
    weights_vector = sum(likelihoods, 2);
    weights = weights_vector / sum(weights_vector);  
end