function [pose, new_particles] = particles_filter(map, particles, vel, sampleTime, ranges, distance_map, resolution)
    disp('Iniciando filtro de partículas...');
    odom_particles = particles + vel' * sampleTime; % Actualizar partículas con la velocidad del robot
    % odom_particles = localization.odometry_model(particles, vel, sampleTime);
    weights = localization.measurement_model_likelihood_field(map, odom_particles, ranges, distance_map, resolution);
    new_particles = localization.resample(odom_particles, weights);
    pose = mean(new_particles, 1);
    disp('Filtro de partículas completado.');
    figure(100); clf;
    show(map); hold on;
    plot(new_particles(:,1), new_particles(:,2), 'b.'); axis equal;
    quiver(new_particles(:,1), new_particles(:,2), cos(new_particles(:,3)), sin(new_particles(:,3)), 0, 'r')
    title('Distribución de partículas después del resampling');
    disp(sum(weights)); % Mostrar suma de pesos para verificar normalización
end