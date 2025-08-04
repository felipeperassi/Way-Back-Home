function [path_points, total_cost, path_length] = astar(resolution, start_world, goal_world, distance_map, occupancy_map)
    % A* - Algoritmo de búsqueda A* para encontrar un camino en un mapa de ocupación.
    %
    % Entradas:
    %   resolution      - Resolución del mapa (celdas por metro)
    %   start_world     - Coordenadas [x, y] del punto de inicio (en metros)
    %   goal_world      - Coordenadas [x, y] del punto objetivo (en metros)
    %   distance_map    - Mapa de distancias al obstáculo más cercano
    %   occupancy_map   - Mapa binario o probabilístico de ocupación
    %
    % Salidas:
    %   path_points     - Camino encontrado en coordenadas del mundo [x, y]
    %   total_cost      - Costo total acumulado del camino
    %   path_length     - Longitud total del camino (en número de celdas)

    %% Algoritmo A*

    disp('Iniciando búsqueda A*...');

    % Dimensiones del mapa de ocupación
    [h, w] = size(occupancy_map);
    
    % Inicialización de estructuras (costos, heurísticas, celdas cerradas, x e y previos)
    costs = ones(h,w)*inf;
    heuristics = zeros(h,w);
    closed_list = zeros(h,w);
    previous_x = zeros(h,w)-1;
    previous_y = zeros(h,w)-1;

    % Conversión de coordenadas del mundo a índices de matriz [fila, columna]
    start = [round(start_world(2)*resolution)+1, round(start_world(1)*resolution)+1];
    goal  = [round(goal_world(2)*resolution)+1,  round(goal_world(1)*resolution)+1];

    % Validación de que el inicio y el objetivo están dentro del mapa
    if any(start < 1) || any(start > [h w]) || any(goal < 1) || any(goal > [h w])
        error('Start o goal están fuera de los límites del mapa.');
    end

    % Inicializa el punto de partida
    parent = start;
    costs(start(1),start(2)) = 0;

    % Bucle principal del algoritmo A*
    while ~isequal(parent, goal)

        % Calcula la lista abierta (costo total estimado)
        closed_mask = closed_list;
        closed_mask(closed_mask==1) = Inf;
        open_list = costs + closed_mask + heuristics;

        % Si no hay más celdas accesibles
        if min(open_list(:)) == Inf
            error('No se encontró un camino válido.');
        end

        % Selecciona el nodo con menor costo total
        [y, x] = find(open_list == min(open_list(:)), 1);
        parent = [y, x];
        closed_list(y,x) = 1;

        % Revisa los vecinos accesibles
        n = planning.neighbors(parent, [h, w]);
        for i = 1:size(n,1)
            child = n(i,:);

            % Calcula el nuevo costo de llegar al vecino
            cost_val = costs(y,x) + planning.edge_cost(parent, child, occupancy_map, distance_map);
            heuristic_val = planning.heuristic(child, goal);

            % Si se mejora el camino al vecino, se actualiza
            if cost_val < costs(child(1), child(2))
                costs(child(1), child(2)) = cost_val;
                heuristics(child(1), child(2)) = heuristic_val;
                previous_x(child(1), child(2)) = x;
                previous_y(child(1), child(2)) = y;
            end
        end
    end

    % Reconstrucción del camino desde el objetivo hacia el inicio
    parent = goal;
    path_points = [];
    path_length = 0;

    while previous_x(parent(1), parent(2)) >= 0

        % Convierte coordenadas del mapa a coordenadas del mundo
        x_world = (parent(2)-1)/resolution;
        y_world = (parent(1)-1)/resolution;

        % Agrega el punto al camino
        path_points = [path_points; x_world, y_world];

        % Actualiza el padre para seguir hacia atrás
        child = [previous_y(parent(1), parent(2)), previous_x(parent(1), parent(2))];
        path_length = path_length + norm(parent - child);
        parent = child;
    end

    % Costo total del camino encontrado
    total_cost = costs(goal(1), goal(2));

    %% Visualización del camino encontrado

    % % Setear la visualización
    % figure(1000); hold on; axis equal;
    % imagesc(occupancy_map); colormap(gray);
    % set(gca, 'YDir', 'normal');
    % title('Ruta A* encontrada');
    % xlabel('X'); ylabel('Y');

    % % Graficar camino
    % plot(path_points(:,1)*resolution, path_points(:,2)*resolution, 'b-', 'LineWidth', 2);

    % % Graficar inicio y meta
    % plot(start_world(1)*resolution, start_world(2)*resolution, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    % plot(goal_world(1)*resolution,  goal_world(2)*resolution,  'ro', 'MarkerSize', 10, 'LineWidth', 2);

    % legend({'Ruta','Inicio','Meta'});
end
