function [pose, new_particles] = particles_filter(map, particles, v_cmd, w_cmd, sampleTime, lidar, ranges)
    disp('Iniciando filtro de partículas...');
    odom_particles = odometry_model(particles, v_cmd, w_cmd, sampleTime);
    weights = measurement_model(odom_particles, lidar, ranges, map);
    new_particles = resample(odom_particles, weights);
    pose = mean(new_particles, 1);
    disp('Filtro de partículas completado.');
    figure(100); clf;
    show(map); hold on;
    plot(new_particles(:,1), new_particles(:,2), 'b.'); axis equal;
    title('Distribución de partículas después del resampling');
end