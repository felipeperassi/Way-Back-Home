function particles = initialize_particles_in_pose(num_particles, pose, map)
    % INITIALIZE_PARTICLES_IN_POSE - Inicializa partículas alrededor de una pose conocida, evitando zonas ocupadas del mapa.
    %
    % Entradas:
    %   num_particles - Número total de partículas a generar
    %   pose          - Pose [x, y, theta] alrededor de la cual se distribuyen las partículas
    %   map           - Mapa de ocupación (objeto binaryOccupancyMap)
    %
    % Salidas:
    %   particles     - Matriz Nx3 con partículas inicializadas en [x, y, theta]

    %% Generación de partículas alrededor de una pose conocida

    % Inicializar matriz de partículas [x, y, theta]
    particles = zeros(num_particles, 3); 
    
    % Desvío dado por el radio del robot
    sigma = 0.1; 
    
    % Generar partículas alrededor de la pose dada, con un pequeño ruido
    % y asegurando que estén en zonas libres del mapa
    count = 0;
    while count < num_particles
        x = pose(1) + randn * sigma;
        y = pose(2) + randn * sigma;
        theta = wrapToPi(pose(3) + randn * deg2rad(2)); 
        if getOccupancy(map, [x, y]) <= 0.1
            count = count + 1;
            particles(count, :) = [x, y, theta];
        end
    end

    %% Visualización de la distribución de partículas

    % figure(250); clf;
    % show(map); hold on;
    % plot(particles(:,1), particles(:,2), 'b.'); axis equal;
    % title('Distribución de partículas en pose conocida');
end