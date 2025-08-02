%% Robot diferencial con lidar
% Robotica Movil - 2025 2c
close all
clear all

verMatlab= ver('MATLAB');       % en MATLAB2020a funciona bien, ajustado para R2016b, los demas a pelearla...

simular_ruido_lidar = false;    %simula datos no validos del lidar real, probar si se la banca
use_roomba=false;               % false para desarrollar usando el simulador, true para conectarse al robot real

%% Roomba
if use_roomba   % si se usa el robot real, se inicializa la conexion    
    rosshutdown
    pause(1)
    ipaddress_core = '192.168.0.102';
    ipaddress_local = '192.168.0.100';  %mi ip en a red TurtleNet
    setenv('ROS_IP', '192.168.0.100');
    setenv('ROS_MASTER_URI', ['http://', ipaddress_core, ':11311']);
    rosinit(ipaddress_core,11311, 'NodeHost', ipaddress_local)
    pause(.5)
    laserSub = rossubscriber('/scan');
    odomSub = rossubscriber('/odom');
    cmdPub = rospublisher('/auto_cmd_vel', 'geometry_msgs/Twist');
    pause(.5) % Esperar a que se registren los canales
    cmdMsg = rosmessage(cmdPub);  
end
    

%% Definicion del robot (disco de diametro = 0.35m)
R = 0.072/2;                % Radio de las ruedas [m]
L = 0.235;                  % Distancia entre ruedas [m]
dd = simulator.DifferentialDrive(R,L); % creacion del Simulador de robot diferencial

%% Creacion del entorno
load mapa_TP_2025a.mat      %carga el mapa como occupancyMap en la variable 'map'
% load mapa_fiuba_1p.mat      %carga el mapa como occupancyMap en la variable 'map'
% load mapa_lae.mat         %mapa viejo para probar cosas

if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
    imagen_mapa = 1-double(imread('mapa_fiuba_1p.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release(1:5)=='(R201'    % Completar con la version que tengan
    %Ni idea que pasa, ver si el truco R2016b funciona
    disp('ver si la compatibilidad R2016b funciona');
else
    disp(['Utilizando MATLAB ', verMatlab.Release]);
end

%% Crear sensor lidar en simulador
lidar = simulator.LidarSensor;
lidar.sensorOffset = [0,0];     % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 3;                %decimar lecturas de lidar acelera el algoritmo
num_scans = 513/scaleFactor;
hokuyo_step_a = deg2rad(-90);
hokuyo_step_c = deg2rad(90);

lidar.scanAngles = linspace(hokuyo_step_a, hokuyo_step_c, num_scans);
lidar.maxRange = 5;

%% Crear visualizacion
viz = simulator.Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Parametros de la Simulacion

simulationDuration = 3*60;          % Duracion total [s]
sampleTime = 0.1;                   % Sample time [s]
initPose = [30, 30, -pi/2];         % Pose inicial (x y theta) del robot simulado (el robot puede arrancar en cualquier lugar valido del mapa)
                                    % Probar iniciar el robot en distintos lugares   
                                    % Medio: [18, 16, pi/2]; Abajo Izq: [6.5; 5; pi/2]; Medio Der: [30; 5; pi];                               
                                  
% Inicializar vectores de tiempo:1010
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

%% generar comandos a modo de ejemplo
vxRef = 0.1*ones(size(tVec));   % Velocidad lineal a ser comandada
wRef = zeros(size(tVec));       % Velocidad angular a ser comandada
wRef(tVec < 5) = -0.1;
wRef(tVec >=7.5) = 0.1;

pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;

%% Simulacion

if verMatlab.Release=='(R2016b)'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end

% Inicializar las particulas
num_init_particles = 1000;                                              % Numero de particulas
particles = localization.initialize_particles(num_init_particles, map); % Inicializar particulas en el mapa

% Mapas de distancias y de ocupación
[distance_map, M] = localization.calculate_distance_map(map, false);                % Calcular mapa de distancias
[distance_map_flipped, M_flipped] = localization.calculate_distance_map(map, true); % Calcular mapa de distancias con Y hacia arriba

% Variables del robot
robot_state = 'localization';   % Estado inicial del robot
var_pose_est = 100 * ones(3,1); % Varianza de pose estimada
v_cmd = 0;                      % Velocidad lineal inicial
w_cmd = 0;                      % Velocidad angular inicial
count_react = 0;                % Contador de reacciones ante obstáculos
goal_world  = [12.5, 15];       % Coordenadas del LAR

for idx = 2:numel(tVec)   

    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.   
    
    %% 
    disp('iteracion: ' + string(idx));
    
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    
    if use_roomba       % para usar con el robot real
        
        % Enviar comando de velocidad en el formato que pide el robot:
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometría
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = double(ranges_full(1:scaleFactor:end));
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        ranges(ranges<0.05)=NaN; % compensación por errores de medicion no identificados a Dic/24
        
        % Obtener pose del robot [x,y,yaw] de datos de odometría (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,idx) = [odompose.Pose.Pose.Position.X + initPose(1); odompose.Pose.Pose.Position.Y+ initPose(2); odomRotation(1)];
    
    else        % para usar el simulador
   
        % Mover el robot segun los comandos generados
        [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v; 0; w]; % velocidades en la terna del robot [vx;vy;w]
        vel = simulator.bodyToWorld(velB, pose(:,idx-1));  % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
        % Tomar nueva medicion del lidar
        ranges = double(lidar(pose(:,idx)));
        if simular_ruido_lidar
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid=rand(length(ranges),1);
            ranges(not_valid<=chance_de_medicion_no_valida)=NaN;
        end
    end
    %%
    % Aca el robot ya ejecutó las velocidades comandadas y devuelve en la
    % variable ranges la medicion del lidar para ser usada y
    % en la variable pose(:,idx) la odometría actual.
    
    %% 
    
    [min_dist, idx_min] = min(ranges, [], 'omitnan');  % Encuentra la distancia mínima y su índice
    [max_dist, idx_max] = max(ranges, [], 'omitnan');  % Encuentra la distancia máxima y su índice
    max_angle = wrapToPi(lidar.scanAngles(idx_max));   % Ángulo máximo del lidar

    obstacle = movement.obstacle_detected(min_dist);   % Detecta si hay un obstáculo cercano

    switch robot_state
        case 'localization' % el robot se localiza en el mapa
            [pose_est, var_pose_est, particles] = localization.particles_filter(map, particles, vel, sampleTime, ranges, distance_map, M);
            [v_cmd, w_cmd, count_react, robot_state] = movement.reactive_movement(v_cmd, w_cmd, max_angle, obstacle, robot_state, count_react, true);
            converged_pose = all(var_pose_est(1:2) <= 0.3);

            if converged_pose % se obtiene la pose estimada y se inicializan menos particulas para quitarle costo computacional
                num_particles = 100;
                new_particles = localization.initialize_particles_in_pose(num_particles, pose_est, map);
                [pose_est, var_pose_est, new_particles] = localization.particles_filter(map, new_particles, vel, sampleTime, ranges, distance_map, M);

                correct_localization = localization.confirm_localization(pose_est, lidar, ranges); % se usa raycasting para confirmar la localización
                if correct_localization
                    disp('Localización confirmada');
                    robot_state = 'calculate_destiny'; % Cambiar estado para calcular el destino
                else
                    disp('Localización no confirmada, reintentando...');
                    particles = localization.initialize_particles(num_init_particles, map);
                    robot_state = 'localization'; % Reintentar localización
                end
            end
        
        case 'calculate_destiny' % calcular el destino y generar la ruta
            [path_points, total_cost, path_length] = planning.astar(map.Resolution, pose_est(1:2), goal_world, distance_map_flipped, M_flipped);
            next_wp_idx = size(path_points, 1);
            robot_state = 'navigation';
        
        case 'navigation' % el robot navega hacia el destino
            [pose_est, var_pose_est, new_particles] = localization.particles_filter(map, new_particles, vel, sampleTime, ranges, distance_map, M);

            if obstacle
                disp('Detecte un obstaculo a ' + string(min_dist));
                robot_state = 'reactive';
                
            else
                disp('Siguiendo ruta planificada');
                [robot_state, v_cmd, w_cmd, next_wp_idx] = movement.follow_path(pose_est, path_points, next_wp_idx);
            end
        
        case 'reactive' % el robot reacciona ante un obstaculo detectado
            [pose_est, var_pose_est, new_particles] = localization.particles_filter(map, new_particles, vel, sampleTime, ranges, distance_map, M);
            [v_cmd, w_cmd, count_react, robot_state] = movement.reactive_movement(v_cmd, w_cmd, max_angle, obstacle, robot_state, count_react, false);
        
        case 'stationary' % el robot se detiene
            v_cmd = 0;
            w_cmd = 0;

    end
    
    disp('Pose estimada:' + string(pose_est));
    disp('Varianza de la pose estimada:' + string(var_pose_est));
        
    %%
    % actualizar visualizacion
    viz(pose(:,idx),ranges)
    waitfor(r);
end

