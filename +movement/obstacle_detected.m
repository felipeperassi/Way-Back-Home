function obstacle = obstacle_detected(min_dist)
    % OBSTACLE_DETECTED - Determina si hay un obstáculo cercano basado en una distancia mínima medida.
    %
    % Entrada:
    %   min_dist - Distancia mínima medida por el LiDAR (en metros)
    %
    % Salida:
    %   obstacle - Booleano que indica si hay un obstáculo cerca o no

    % Si la distancia mínima es menor o igual a 0.3 metros, se considera que hay un obstáculo
    if min_dist <= 0.3
        obstacle = true;
    else
        obstacle = false;
    end
end