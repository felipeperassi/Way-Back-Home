function new_particles = odometry_model(particles, vel, sampleTime)
    motion_noise = [0.5, deg2rad(3)]; % [ruido_v, ruido_w]
    new_particles = particles;
    N = size(particles, 1);

    for i = 1:N
        v = vel(1) + randn * motion_noise(1);
        w = vel(2) + randn * motion_noise(2);

        theta = particles(i, 3);
        dx = v * sampleTime * cos(theta);
        dy = v * sampleTime * sin(theta);
        dtheta = w * sampleTime;

        new_particles(i,1) = particles(i,1) + dx;
        new_particles(i,2) = particles(i,2) + dy;
        new_particles(i,3) = wrapToPi(theta + dtheta);
    end
end