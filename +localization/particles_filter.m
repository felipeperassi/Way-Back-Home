function [mean_pose, var_pose, new_particles] = particles_filter(map, particles, vel, sampleTime, ranges, angles, distance_map, occupancy_map)
    % PARTICLES_FILTER - Realiza la localización del robot mediante un filtro de partículas.
    %
    % Entradas:
    %   map             - Mapa de ocupación (objeto binaryOccupancyMap)
    %   particles       - Matriz Nx3 con las partículas actuales [x, y, theta]
    %   vel             - Vector de velocidad del robot [vx; vy; vtheta]
    %   sampleTime      - Paso de tiempo de integración (en segundos)
    %   ranges          - Vector de distancias medidas por el LiDAR
    %   angles          - Vector de ángulos correspondientes a las mediciones LiDAR
    %   distance_map    - Mapa de distancias usado para la comparación de lecturas
    %   occupancy_map   - Mapa de ocupación para la evaluación de obstáculos
    %
    % Salidas:
    %   mean_pose       - Estimación de la pose del robot [x, y, theta] (media de partículas)
    %   var_pose        - Varianza de las partículas en cada dimensión [var_x, var_y, var_theta]
    %   new_particles   - Conjunto de partículas actualizado luego del resampleo
    
    %% Filtro de partículas para localización del robot

    disp('Iniciando filtro de partículas...');

    % Actualizar partículas con la velocidad del robot
    odom_particles = particles + vel' * sampleTime; 

    % Calcular pesos de las partículas
    weights = localization.measurement_model_likelihood_field(map, odom_particles, ranges, angles, distance_map, occupancy_map);

    % Resampleo de partículas basado en los pesos
    new_particles = localization.resample(odom_particles, weights); 
    
    % Calcular pose estimada como la media de las partículas
    mean_pose = mean(new_particles, 1);    
    mean_pose(3) = wrapToPi(mean_pose(3));  

    % Calcular varianza de las partículas
    var_pose = var(new_particles, 0, 1);    

    disp('Filtro de partículas completado.');

    %% Visualización de la distribución de partículas

    figure(100); clf;
    show(map); hold on;
    plot(new_particles(:,1), new_particles(:,2), 'b.'); axis equal;
    quiver(new_particles(:,1), new_particles(:,2), cos(new_particles(:,3)), sin(new_particles(:,3)), 0, 'r')
    title('Distribución de partículas después del resampling');
end