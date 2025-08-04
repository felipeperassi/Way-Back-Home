function cost = edge_cost(parent, child, map, distance_map)
    % EDGE_COST - Calcula el costo de transición entre dos celdas del mapa.
    %
    % Entradas:
    %   parent         - Coordenadas de la celda origen [row, col]
    %   child          - Coordenadas de la celda destino [row, col]
    %   map            - Mapa de ocupación (matriz de valores entre 0 y 1)
    %   distance_map   - Mapa de distancias a los obstáculos más cercanos (en celdas)
    %
    % Salidas:
    %   cost           - Costo de moverse de 'parent' a 'child'. Si la celda está ocupada, retorna un valor muy alto.

    % Parámetro de umbral de ocupación
    threshold = 0.01;

    % Valor de ocupación en la celda destino
    occ = map(child(1), child(2));

    % Obtener distancia a obstáculo en esa celda
    d = distance_map(child(1), child(2));

    if occ < threshold
        % La celda está libre. Se penaliza por cercanía a obstáculos.
        if d < 20  
            safety_penalty = 100 * (1 / (d + 0.1));  % evitar división por 0
        
        % Lejana a obstáculos, penalización baja
        else
            safety_penalty = 0;
        end

        % Distancia entre celdas
        dist = sqrt((child(1) - parent(1))^2 + (child(2) - parent(2))^2);

        % Costo: distancia + penalización
        cost = dist + safety_penalty;
    else

        % La celda está ocupada: se penaliza fuertemente
        cost = 1e12;
    end
end