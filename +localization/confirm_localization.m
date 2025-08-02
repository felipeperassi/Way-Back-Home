function [correct_localization] = confirm_localization(pose_est, lidar, ranges)

    % Simular el rango de medición del lidar basado en la pose estimada
    simulated_ranges = lidar(pose_est');

    % Calcular la diferencia entre las mediciones reales y simuladas
    diff_ranges = abs(ranges - simulated_ranges);
    diff_ranges = diff_ranges(~isnan(diff_ranges));
    if sum(diff_ranges < 0.15) > (0.5 * length(diff_ranges)) % Al menos el 50% de las distancias deben ser menores a 0.2
        correct_localization = true;
    else
        correct_localization = false;
    end

    % n_particles = size(particles, 1);
    % bool_vector = false(n_particles, 1);

    % for i = 1:n_particles
    %     simulated_ranges = lidar(particles(i, :)');

    %     diff_ranges = abs(ranges - simulated_ranges);
    %     diff_ranges = diff_ranges(~isnan(diff_ranges));
    %     if sum(diff_ranges < 0.1) > (0.5 * length(diff_ranges)) % Al menos el 50% de las distancias deben ser menores a 0.3
    %         bool_vector(i) = true;
    %     else 
    %         bool_vector(i) = false;
    %     end
    % end

    % correct_localization = sum(bool_vector) > (0.5 * size(bool_vector, 1)); % Al menos el 50% de las partículas deben coincidir con las distancias medidas
end