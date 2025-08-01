function [path_points, total_cost, path_length] = astar(resolution, start_world, goal_world, distance_map, M)
    
    if nargin < 6
        plot_flag = false; % por defecto, no graficar
    end

    % Dimensiones
    [h, w] = size(M);
    
    % Inicialización
    costs = ones(h,w)*inf;
    heuristics = zeros(h,w);
    closed_list = zeros(h,w);
    previous_x = zeros(h,w)-1;
    previous_y = zeros(h,w)-1;

    % Convertir coordenadas del mundo a índices
    start = [round(start_world(2)*resolution)+1, round(start_world(1)*resolution)+1];  % [row, col]
    goal  = [round(goal_world(2)*resolution)+1,  round(goal_world(1)*resolution)+1];

    % Validación de límites
    if any(start < 1) || any(start > [h w]) || any(goal < 1) || any(goal > [h w])
        error('Start o goal están fuera de los límites del mapa.');
    end

    % Iniciar búsqueda
    parent = start;
    costs(start(1),start(2)) = 0;

    disp('Iniciando búsqueda A*...');

    while ~isequal(parent, goal)
        closed_mask = closed_list;
        closed_mask(closed_mask==1) = Inf;
        open_list = costs + closed_mask + heuristics;

        if min(open_list(:)) == Inf
            error('No se encontró un camino válido.');
        end

        [y, x] = find(open_list == min(open_list(:)), 1);
        parent = [y, x];
        closed_list(y,x) = 1;

        n = planning.neighbors(parent, [h, w]);
        for i = 1:size(n,1)
            child = n(i,:);
            cost_val = costs(y,x) + planning.edge_cost(parent, child, M, distance_map);
            heuristic_val = planning.heuristic(child, goal);
            if cost_val < costs(child(1), child(2))
                costs(child(1), child(2)) = cost_val;
                heuristics(child(1), child(2)) = heuristic_val;
                previous_x(child(1), child(2)) = x;
                previous_y(child(1), child(2)) = y;
            end
        end
    end

    % Reconstruir camino
    parent = goal;
    path_points = [];
    path_length = 0;

    while previous_x(parent(1), parent(2)) >= 0
        x_world = (parent(2)-1)/resolution;
        y_world = (parent(1)-1)/resolution;
        path_points = [path_points; x_world, y_world];
        child = [previous_y(parent(1), parent(2)), previous_x(parent(1), parent(2))];
        path_length = path_length + norm(parent - child);
        parent = child;
    end

    total_cost = costs(goal(1), goal(2));


        figure(1000); hold on; axis equal;
        imagesc(M); colormap(gray);
        set(gca, 'YDir', 'normal');
        title('Ruta A* encontrada');
        xlabel('X'); ylabel('Y');

        % Graficar camino
        plot(path_points(:,1)*resolution, path_points(:,2)*resolution, 'b-', 'LineWidth', 2);

        % Graficar inicio y meta
        plot(start_world(1)*resolution, start_world(2)*resolution, 'go', 'MarkerSize', 10, 'LineWidth', 2);
        plot(goal_world(1)*resolution,  goal_world(2)*resolution,  'ro', 'MarkerSize', 10, 'LineWidth', 2);

        legend({'Ruta','Inicio','Meta'});
end
