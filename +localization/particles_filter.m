function [mean_pose, var_pose, new_particles] = particles_filter(map, particles, vel, sampleTime, ranges, distance_map, M)
    disp('Iniciando filtro de partículas...');
    odom_particles = particles + vel' * sampleTime; % Actualizar partículas con la velocidad del robot
    weights = localization.measurement_model_likelihood_field(map, odom_particles, ranges, distance_map, M); % Calcular pesos de las partículas
    new_particles = localization.resample(odom_particles, weights); % Resample de partículas basado en los pesos
    
    mean_pose = mean(new_particles, 1); % Calcular pose estimada como la media de las partículas
    mean_pose(3) = wrapToPi(mean_pose(3));  % Asegurar que el ángulo esté en el rango [-pi, pi]

    var_pose = var(new_particles, 0, 1); % Calcular varianza de las partículas
    disp('Filtro de partículas completado.');
    figure(100); clf;
    show(map); hold on;
    plot(new_particles(:,1), new_particles(:,2), 'b.'); axis equal;
    quiver(new_particles(:,1), new_particles(:,2), cos(new_particles(:,3)), sin(new_particles(:,3)), 0, 'r')
    title('Distribución de partículas después del resampling');
    disp(sum(weights)); % Mostrar suma de pesos para verificar normalización
end