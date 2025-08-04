function [distance_map, occupancy_map] = calculate_distance_map(map, flip_y)
    % CALCULATE_DISTANCE_MAP - Genera un mapa de distancias a obstáculos a partir de un mapa de ocupación.
    %
    % Entradas:
    %   map            - Mapa de ocupación (objeto binaryOccupancyMap)
    %   flip_y         - Booleano que indica si se debe voltear verticalmente el mapa (eje Y)
    %
    % Salidas:
    %   distance_map   - Mapa con la distancia euclidiana a los obstáculos más cercanos
    %                    (en metros si flip_y es false, en celdas si es true)
    %   occupancy_map  - Matriz de ocupación obtenida del mapa (valores entre 0 y 1)

    % Obtener la matriz de ocupación del mapa
    occupancy_map = getOccupancy(map);
    
    % Si se indica, voltear la matriz verticalmente (para A*)
    if flip_y
        occupancy_map = flipud(occupancy_map);
    end

    % Crear un mapa binario: 1 si es obstáculo, 0 si es libre (umbral = 0.195)
    binary_map = occupancy_map > 0.195;

    % Calcular la distancia euclidiana desde cada celda libre al obstáculo más cercano
    distance_map = bwdist(binary_map, "euclidean");
    
    % Si no se volteó el mapa, convertir la distancia de celdas a metros
    if ~flip_y
        distance_map = distance_map / map.Resolution;
    end
end