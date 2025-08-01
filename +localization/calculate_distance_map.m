function [distance_map, M] = calculate_distance_map(map)
    M = getOccupancy(map);
    binary_map = M > 0.1;
    cell_distance_map = bwdist(binary_map, "euclidean");
    distance_map = cell_distance_map / map.Resolution;
    
    % Comparar ambos mapas con la misma escala de colores
    % figure;
    % subplot(1,2,1);
    % imagesc(cell_distance_map);
    % axis equal tight; colorbar;
    % title(['Distance Map (pixels) - Max: ', num2str(max(cell_distance_map(:)))]);
    
    % subplot(1,2,2);
    % imagesc(distance_map);
    % axis equal tight; colorbar;
    % title(['Distance Map (meters) - Max: ', num2str(max(distance_map(:)))]);
    
    % fprintf('Resolution: %.3f m/pixel\n', map.Resolution);
    % fprintf('Max distance in pixels: %.2f\n', max(cell_distance_map(:)));
    % fprintf('Max distance in meters: %.2f\n', max(distance_map(:)));
end