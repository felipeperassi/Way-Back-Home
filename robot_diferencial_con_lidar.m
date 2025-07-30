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
dd = DifferentialDrive(R,L); % creacion del Simulador de robot diferencial

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
lidar = LidarSensor;
lidar.sensorOffset = [0,0];     % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 3;                %decimar lecturas de lidar acelera el algoritmo
num_scans = 513/scaleFactor;
hokuyo_step_a = deg2rad(-90);
hokuyo_step_c = deg2rad(90);

lidar.scanAngles = linspace(hokuyo_step_a,hokuyo_step_c,num_scans);
lidar.maxRange = 5;

%% Crear visualizacion
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Parametros de la Simulacion

simulationDuration = 3*60; %3*60;     % Duracion total [s]
sampleTime = 0.1;                   % Sample time [s]
initPose = [30; 6; pi];           % Pose inicial (x y theta) del robot simulado (el robot puede arrancar en cualquier lugar valido del mapa)
                                    %  probar iniciar el robot en distintos lugares                                  
                                  
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
num_particles = 10000; % Numero de particulas
particles = localization.initialize_particles(num_particles, map); % Inicializar particulas en el mapa
[distance_map, M] = localization.calculate_distance_map(map); % Calcular mapa de distancias

for idx = 2:numel(tVec)   

    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.   
    
    %% ---- COMPLETAR ACA: ----
    disp('iteracion: ' + string(idx));
    if idx == 2
        robot_state = 'localization';
        v_cmd = 0;
        w_cmd = 0;   
        count_react = 0;
        goal_world  = [12.5, 15];  % Coordenadas del LAR
    end

    % ---- fin del COMPLETAR ACA ----
    
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
        vel = bodyToWorld(velB, pose(:,idx-1));  % Conversion de la terna del robot a la global
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
    
    %% ---- COMPLETAR ACA: ----
        % hacer algo con la medicion del lidar (ranges) y con el robot_state
        % actual de la odometria ( pose(:,idx) ) que se utilizará en la
        % proxima iteración para la generacion de comandos de velocidad
        % ...
    
    [min_dist, idx_min] = min(ranges, [], 'omitnan');  
    [max_dist, idx_max] = max(ranges, [], 'omitnan'); 
    max_angle = wrapToPi(lidar.scanAngles(idx_max));

    obstacle = obstacle_detected(min_dist);
    
    switch robot_state
        case 'localization' % el robot se localiza en el mapa

            if idx <= 4 % primeras iteraciones, se generan muchas partículas
                [pose_est, particles] = localization.particles_filter(map, particles, vel, sampleTime, ranges, distance_map);

            elseif idx == 5 % se obtiene la pose estimada y se inicializan menos particulas para quitarle costo computacional
                num_particles = 25;
                new_particles = localization.initialize_particles_in_pose(num_particles, pose_est, map);
                [pose_est, new_particles] = localization.particles_filter(map, new_particles, vel, sampleTime, ranges, distance_map);

                robot_state = 'calculate_destiny'; 
            end
        
        case 'calculate_destiny' % calcular el destino y generar la ruta
            [path_points, total_cost, path_length] = astar(map.Resolution, pose(1:2, idx-1), goal_world, distance_map, M);
            next_wp_idx = size(path_points, 1);
            robot_state = 'navigation';
        
        case 'navigation' % el robot navega hacia el destino
            [pose_est, new_particles] = localization.particles_filter(map, new_particles, vel, sampleTime, ranges, distance_map);

            if obstacle
                disp('Detecte un obstaculo a ' + string(min_dist));
                robot_state = 'reactive';
            else
                disp('Siguiendo ruta planificada');
                % Extraer pose actual estimada para el control
                x = pose(1, idx-1);
                y = pose(2, idx-1);
                theta = pose(3, idx-1);
                
                goal_point = path_points(1,:);  % Objetivo final
                dist_to_goal = norm([x, y] - goal_point);

                if dist_to_goal < 0.5
                    robot_state = 'stationary';
                else
                    goalReached = false;
                    while ~goalReached && next_wp_idx > 1
                        wp = path_points(next_wp_idx, :);
                        dx = wp(1) - x;
                        dy = wp(2) - y;
                        dist_to_wp = norm([dx dy]);
                        if dist_to_wp < 0.4
                            next_wp_idx = next_wp_idx - 1;
                        else
                            goalReached = true;
                        end
                    end

                    wp = path_points(next_wp_idx, :);
                    dx = wp(1) - x;
                    dy = wp(2) - y;
                    angle_to_wp = atan2(dy, dx);
                    angle_diff = wrapToPi(angle_to_wp - theta);
                    
                    % Control simple
                    v_cmd = 0.5; 
                    w_cmd = 1.5 * angle_diff;
                end
            end
        
        case 'reactive' % el robot reacciona ante un obstaculo detectado
            [pose_est, new_particles] = localization.particles_filter(map, new_particles, vel, sampleTime, ranges, distance_map);

            if obstacle && count_react < 10
                disp('Obstaculo detectado, deteniendo robot');
                v_cmd = 0;
                w_cmd = 0.5 * max_angle; 
                count_react = count_react + 1;
            
            elseif obstacle && count_react >= 10 && count_react < 15
                disp('Obstaculo detectado, girando para evitar');
                v_cmd = 0.05;
                w_cmd = 0.5 * max_angle;
                count_react = count_react + 1;

            elseif obstacle && count_react >= 15
                count_react = 0;
                robot_state = 'calculate_destiny';

            else
                count_react = 0;
                robot_state = 'calculate_destiny'; 
            end
        
        case 'stationary' % el robot se detiene
            v_cmd = 0;
            w_cmd = 0;

        w_cmd = max(min(w_cmd, 0.5), -0.5);
        v_cmd = max(min(v_cmd, 0.1), -0.1);
    end
    
    disp('Pose estimada:' + string(pose_est));

    % ---- Fin del COMPLETAR ACA ----
        
    %%
    % actualizar visualizacion
    viz(pose(:,idx),ranges)
    waitfor(r);
end

