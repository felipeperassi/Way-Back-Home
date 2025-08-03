function [distance_map, M] = calculate_distance_map(map, flip_y)
    if flip_y
        M = occupancyMatrix(map);
        M = flipud(M); 
    else
        M = getOccupancy(map);
    end

    binary_map = M > 0.195;
    distance_map = bwdist(binary_map, "euclidean");
    
    if ~flip_y
        distance_map = distance_map / map.Resolution;
    end
end