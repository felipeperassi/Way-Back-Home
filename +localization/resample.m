function new_particles = resample(particles, weights)
    % RESAMPLE - Realiza el remuestreo de partículas según el método SUS (Stochastic Universal Sampling) y agregando ruido gaussiano.
    %
    % Entradas:
    %   particles   - Matriz de partículas N x 3 con las poses de las partículas [x, y, theta]
    %   weights     - Vector con los pesos normalizados de cada partícula
    %
    % Salidas:
    %   new_particles - Matriz N x 3 con las nuevas partículas generadas tras el resampleo

    % Inicializar la matriz de nuevas partículas
    new_particles = zeros(size(particles));

    % Número de partículas y dimensiones
    M = size(particles, 1);
    dim = size(particles, 2);

    % Desviaciones estándar del ruido gaussiano agregado a cada partícula
    % Para [x, y] se usa 0.167 m por el radio del robot y 5 grados para theta
    sigma = [0.167, 0.167, deg2rad(5)];

    % Cálculo de la suma acumulada de los pesos, se asegura que sume 1
    cumulative_sum = cumsum(weights);
    cumulative_sum(end) = 1.0;

    % Generación de punteros equiespaciados para el resampleo
    step = 1 / M;
    start = rand * step;
    pointers = start + (0:M-1)' * step;

    % Realizar el remuestreo de partículas
    i = 1;
    for m = 1:M
        while pointers(m) > cumulative_sum(i)
            i = i + 1;
        end
        new_particles(m, :) = particles(i, :) + randn(1, dim) .* sigma;
    end
end