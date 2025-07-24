function new_particles = resample(particles, weights)
    new_particles = zeros(size(particles));
    
    M = size(particles, 1);

    cumulative_sum = cumsum(weights);
    cumulative_sum(end) = 1.0; % Asegurar que suma exactamente 1
    
    step = 1 / M;
    start = rand * step;
    pointers = start + (0:M-1)' * step;

    i = 1;
    for m = 1:M
        while pointers(m) > cumulative_sum(i)
            i = i + 1;
        end
        new_particles(m, :) = particles(i, :);
    end
end