function new_particles = odometry_model(particles, v_cmd, w_cmd, sampleTime)
    motion_noise = [0.01, deg2rad(1)]; % [ruido_v, ruido_w]
    new_particles = particles;
    N = size(particles, 1);

    for i = 1:N
        v = v_cmd + randn * motion_noise(1);
        w = w_cmd + randn * motion_noise(2);

        theta = particles(i, 3);
        dx = v * sampleTime * cos(theta);
        dy = v * sampleTime * sin(theta);
        dtheta = w * sampleTime;

        new_particles(i,1) = particles(i,1) + dx;
        new_particles(i,2) = particles(i,2) + dy;
        new_particles(i,3) = wrapToPi(theta + dtheta);
    end
end