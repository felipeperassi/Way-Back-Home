function particles = initialize_particles_in_pose(num_particles, pose, map)
    particles = zeros(num_particles, 3); % [x, y, theta]
    count = 0;
    sigma = 0.1; % Desviación por el radio del robot
    
    while count < num_particles
        x = pose(1) + randn * sigma;
        y = pose(2) + randn * sigma;
        theta = wrapToPi(pose(3) + randn * deg2rad(2)); 
        if getOccupancy(map, [x, y]) <= 0.1 % Solo zonas libres
            count = count + 1;
            particles(count, :) = [x, y, theta];
        end
    end
    figure(); clf;
    show(map); hold on;
    plot(particles(:,1), particles(:,2), 'b.'); axis equal;
    title('Distribución inicial de partículas');
end