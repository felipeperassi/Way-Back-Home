function [distance_map, M] = calculate_distance_map(map)
    M = occupancyMatrix(map);
    M = flipud(M);  % Y hacia arriba
    distance_map = bwdist(M > 0.1);
    % Plot the distance map
    figure;
    imagesc(distance_map);
    axis equal tight;
    colorbar;
    title('Distance Map');
end
