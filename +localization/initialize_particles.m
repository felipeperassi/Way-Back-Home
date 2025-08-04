function particles = initialize_particles(num_particles, map)
    % INITIALIZE_PARTICLES - Genera una distribución inicial de partículas en zonas libres del mapa.
    %
    % Entradas:
    %   num_particles - Cantidad total de partículas a generar
    %   map           - Mapa de ocupación (objeto binaryOccupancyMap)
    %
    % Salidas:
    %   particles     - Matriz de partículas Nx3 con pose [x, y, theta] distribuidas aleatoriamente en zonas libres

    %% Generación de partículas aleatorias

    % Inicializar matriz de partículas [x, y, theta]
    particles = zeros(num_particles, 3);  
    
    % Obtener los límites del mapa (en metros)
    x_limits = map.XWorldLimits;
    y_limits = map.YWorldLimits;

    % Generar partículas aleatorias dentro del mapa, solo en celdas libres
    count = 0;
    while count < num_particles
        x = unifrnd(x_limits(1), x_limits(2));
        y = unifrnd(y_limits(1), y_limits(2));
        if getOccupancy(map, [x, y]) <= 0.1 
            count = count + 1;
            particles(count, :) = [x, y, unifrnd(-pi, pi)];
        end
    end

    %% Visualización de la distribución inicial de partículas
    
    figure(300); clf;
    show(map); hold on;
    plot(particles(:,1), particles(:,2), 'b.'); axis equal;
    title('Distribución inicial de partículas');
end