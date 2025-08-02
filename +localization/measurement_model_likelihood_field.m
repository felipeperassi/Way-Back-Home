function weights = measurement_model_likelihood_field(map, particles, ranges, distance_map, M)
    explored_map = (M < 0.195) | (M > 0.65);
    n_particles = size(particles, 1);
    n_ray = length(ranges);
    
    alphas = linspace(-pi / 2, pi / 2, n_ray);
    idx_valid_ranges = ~isnan(ranges);
    valid_ranges = ranges(idx_valid_ranges);
    valid_alphas = alphas(idx_valid_ranges);

    sigma = 0.02;  % Desviación estándar del modelo gaussiano, se corresponde con el ruido del sensor

    x_ray = particles(:, 1) + cos(particles(:, 3) + valid_alphas) .* valid_ranges';
    y_ray = particles(:, 2) + sin(particles(:, 3) + valid_alphas) .* valid_ranges';

    dist_matrix = world2grid(map, [x_ray(:), y_ray(:)]);
    row = dist_matrix(:, 1);
    col = dist_matrix(:, 2);

    valid_rows = row >= 1 & row <= size(distance_map, 1);
    valid_cols = col >= 1 & col <= size(distance_map, 2);
    not_nan = ~isnan(row) & ~isnan(col);
    
    valid_mask = valid_rows & valid_cols & not_nan;
    row = row(valid_mask);
    col = col(valid_mask);

    idx_distance_map = sub2ind(size(distance_map), row, col);

    dist_vector = inf(n_particles, n_ray);
    dist_vector(valid_mask) = distance_map(idx_distance_map);

    explored_mask = false(n_particles, n_ray);
    explored_mask(valid_mask) = explored_map(idx_distance_map);
    dist_vector(~explored_mask) = NaN;
 
    likelihoods = exp(-0.5 * (dist_vector / sigma).^2);
    likelihoods(isnan(likelihoods)) = 0; 

    weights = sum(likelihoods, 2);
    weights = weights + 1e-500;  
    weights = weights / sum(weights);  
end