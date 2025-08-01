function cost = edge_cost(parent, child, map, distanceMap)
    % Parámetro de umbral de ocupación (para bloqueo)
    threshold = 0.01;

    occ = map(child(1), child(2));

    % Obtener distancia a obstáculo en esa celda
    d = distanceMap(child(1), child(2));

    if occ < threshold
        % Penalizar inversamente proporcional a la distancia a obstáculo
        % A mayor cercanía, mayor penalización
        if d < 20   % zona de seguridad: menos de 3 celdas (~60cm si resolución = 5)
            safety_penalty = 100 * (1 / (d + 0.1));  % evitar división por 0
        else
            safety_penalty = 0;
        end

        % Distancia entre celdas
        dist = sqrt((child(1) - parent(1))^2 + (child(2) - parent(2))^2);

        cost = dist + safety_penalty;
    else
        cost = 1e12;
    end
end