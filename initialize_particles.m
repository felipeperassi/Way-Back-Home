function particles = initialize_particles(num_particles, map)
    particles = zeros(num_particles, 3); % [x, y, theta]
    count = 0;
    x_limits = map.XWorldLimits;
    y_limits = map.YWorldLimits;
    while count < num_particles
        x = unifrnd(x_limits(1), x_limits(2));
        y = unifrnd(y_limits(1), y_limits(2));
        if getOccupancy(map, [x, y]) <= 0.1 % Solo zonas libres
            count = count + 1;
            particles(count, :) = [x, y, unifrnd(-pi, pi)];
        end
    end
    figure(); clf;
    show(map); hold on;
    plot(particles(:,1), particles(:,2), 'b.'); axis equal;
    title('Distribución inicial de partículas');
end