function [correct_localization] = confirm_localization(particles, lidar, ranges)
    n_particles = size(particles, 1);
    bool_vector = false(n_particles, 1);

    for i = 1:n_particles
        simulated_ranges = lidar(particles(i, :)');

        diff_ranges = abs(ranges - simulated_ranges);
        diff_ranges = diff_ranges(~isnan(diff_ranges));
        if sum(diff_ranges < 0.5) > (0.5 * length(diff_ranges)) % Al menos el 50% de las distancias deben ser menores a 0.5
            bool_vector(i) = true;
        else 
            bool_vector(i) = false;
        end
    end

    correct_localization = sum(bool_vector) > (0.5 * size(bool_vector, 1)); % Al menos el 50% de las partículas deben coincidir con las distancias medidas
end