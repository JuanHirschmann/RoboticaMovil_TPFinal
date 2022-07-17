%% Robot diferencial con lidar
% Robotica Movil - 2022 1c
% Funciona con MATLAB R2018a
close all
clear all

SIMULATE_LIDAR_NOISE = false; %simula datos no validos del lidar real, probar si se la banca
USE_ROOMBA=false;  % false para desarrollar usando el simulador, true para conectarse al robot real

%% Roomba
if USE_ROOMBA   % si se usa el robot real, se inicializa la conexion    
    rosshutdown
    pause(1)
    ipaddress_core = '192.168.0.101';
    ipaddress_local = '192.168.0.100';  %mi ip en a red TurtleNet
    setenv('ROS_IP', '192.168.0.100');
    setenv('ROS_MASTER_URI', ['http://', ipaddress_core, ':11311']);
    rosinit(ipaddress_core,11311, 'NodeHost', ipaddress_local)
    pause(1)
    laserSub = rossubscriber('/scan');
    odomSub = rossubscriber('/odom');
    cmdPub = rospublisher('/auto_cmd_vel', 'geometry_msgs/Twist');
    pause(1) % Esperar a que se registren los canales
    cmdMsg = rosmessage(cmdPub);  
end
    

%% Definicion del robot (disco de diametro = 0.35m)
WHEEL_RADIUS = 0.072/2;                % Radio de las ruedas [m]
L = 0.235;                  % Distancia entre ruedas [m]
diff_drive_obj = DifferentialDrive(WHEEL_RADIUS,L); % creacion del Simulador de robot diferencial

%% Creacion del entorno
MAP_IMG = 1-double(imread('mapa_2022_1c.tiff'))/255;
MAP = robotics.OccupancyGrid(MAP_IMG, 25);
%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = [0,0];   % Posicion del sensor en el robot (asumiendo mundo 2D)
DOWNSAMPLE_FACTOR = 5;                %decimar lecturas de lidar acelera el algoritmo
NUM_SCANS = 513/DOWNSAMPLE_FACTOR;
LIDAR_ANGLE_START = deg2rad(-90);
LIDAR_ANGLE_END = deg2rad(90);
MAX_RANGE=10;
lidar.scanAngles = linspace(LIDAR_ANGLE_START,LIDAR_ANGLE_END,NUM_SCANS);
lidar.maxRange = MAX_RANGE;

%% Crear visualizacion
visualizer = Visualizer2D();

visualizer.hasWaypoints=true;
visualizer.mapName = 'MAP';
attachLidarSensor(visualizer,lidar);
release(visualizer);
%% Parametros de la Simulacion

SIMULATION_DURATION = 3*60;          % Duracion total [s]
SAMPLE_TIME = 0.1;                   % Sample time [s]
INIT_POS = [2.75,2.25,0];%random_empty_point(MAP,[5,1],[5,1]);%[2.5; 1.5; -pi/2];         % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)
WAYPOINTS=[1.5,1.3;4.3,2.1;];
% Inicializar vectores de tiempo, entrada y pose
time_vec = 0:SAMPLE_TIME:SIMULATION_DURATION;         % Vector de Tiempo para duracion total
LOCATION_TIME=20; %20 segundos para ubicarse
LOCATION_ITERATION=int32(20/SAMPLE_TIME); %Iteraciones hasta ubicarse
%% generar comandos a modo de ejemplo
vxRef = [zeros(LOCATION_ITERATION,1);0.05*ones(length(time_vec),1)];   % Velocidad lineal a ser comandada
wRef = [0.5*ones(LOCATION_ITERATION,1);0.05*zeros(length(time_vec),1)];       % Velocidad angular a ser comandada
%wRef(time_vec < 5) = -0.2;
%wRef(time_vec >=7.5) = 0.2;

pose = zeros(3,numel(time_vec));    % Inicializar matriz de pose
pose(:,1) = INIT_POS;

%% Simulacion

robot_sample_rate = robotics.Rate(1/SAMPLE_TIME); %Para Matlab R2018b e inferiores

%Inicializo filtro de partículas

PARTICLES_NUM=1000; %Mas alla de 2000 se pone espeso
POSITION_LIMITS=[5,1;4,1;pi,-pi];
X_LIMS=[5,1];
Y_LIMS=[5,0];
PARTICLE_FILTER_RESAMPLING_INTERVAL=1;  %Remuestrea cada 2 actualizaciones de odometría
PREDICTION_INTERVAL=25;
OUTLIERS_PCT=0.05;
particle_filter=robotics.ParticleFilter; %Creo objeto filtro de partículas
particle_filter.StateEstimationMethod='mean'; %Tomo el promedio de las partículas como mi estado mas probable
particle_filter.StateTransitionFcn=@movement_model; %Función para actualizar la odometría
particle_filter.MeasurementLikelihoodFcn=@measurement_model; %Función del modelo de medición
particle_filter.ResamplingMethod='systematic'; % Remuestreo por SUS
particle_filter.ResamplingPolicy.TriggerMethod='interval'; %Por cantidad de particulas efectivas
particle_filter.ResamplingPolicy.SamplingInterval=PARTICLE_FILTER_RESAMPLING_INTERVAL;
initialize(particle_filter,PARTICLES_NUM,POSITION_LIMITS)

%particle_filter.predict(velocidad_linear,velocidad_angular,delta_t)
%particle_filter.correct(medicion,mapa)

for time_step = 2:length(time_vec) % Itera sobre todo el tiempo de simulación

    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.   
    v_cmd = vxRef(time_step-1);   % estas velocidades estan como ejemplo ...
    w_cmd = wRef(time_step-1);    %      ... para que el robot haga algo.
    %% COMPLETAR ACA:
        % generar velocidades para este timestep
            %% COMPLETAR ACA:
        % generar velocidades para este timestep
        % ACA TIENE QUE IR EL A*
        % fin del COMPLETAR ACA
    start = WAYPOINTS(3,:);
    goalA = WAYPOINTS(1,:);
    goalB = WAYPOINTS(2,:);
    planner = robotics.PRM(MAP);
    theta=INIT_POS(3);
    
%    planner.Map = MAP;
    xyA = findpath(planner,start,goalA);    
    xyB = findpath(planner,goalA,goalB);
   
    velA = zeros(2,size(xyA,1)-1);
    velB = zeros(2,size(xyB,1)-1);
    
    for i = 2:size(xyA,1)
        j=i-1;
        mu = (1/2) * ( ( (xyA(j,1)-xyA(i,1))*cos(theta) + (xyA(j,2)-xyA(i,2))*sin(theta) ) / ((xyA(j,2)-xyA(i,2))*cos(theta) - (xyA(j,1)-xyA(i,1))*sin(theta)) );
        xstar = (xyA(j,1) + xyA(i,1))/2 + mu*(xyA(j,2)-xyA(i,2));
        ystar = (xyA(j,2)+ xyA(i,2))/2 + mu*(xyA(j,1) - xyA(i,1));
        rstar = sqrt((xyA(j,1) - xyA(i,1))^2 + (xyA(j,2)-xyA(i,2))^2 );
        deltatheta = atan2( xyA(i,2) - ystar , xyA(j,1) - xstar ) - atan2( xyA(j,2) - ystar , xyA(j,1) - xstar );
        theta = theta + deltatheta;
        velA(2,j) = deltatheta/SAMPLE_TIME; %w
        velA(1,j) = rstar * velA(2,j); %v     
    end
    
    for i = 2:size(xyB,1)
        j=i-1;
        mu = (1/2) * ( ( (xyB(j,1)-xyB(i,1))*cos(theta) + (xyB(j,2)-xyB(i,2))*sin(theta) ) / ((xyB(j,2)-xyB(i,2))*cos(theta) - (xyB(j,1)-xyB(i,1))*sin(theta)) );
        xstar = (xyB(j,1) + xyB(i,1))/2 + mu*(xyB(j,2)-xyB(i,2));
        ystar = (xyB(j,2)+ xyB(i,2))/2 + mu*(xyB(j,1) - xyB(i,1));
        rstar = sqrt((xyB(j,1) - xyB(i,1))^2 + (xyB(j,2)-xyB(i,2))^2 );
        deltatheta = atan2( xyB(i,2) - ystar , xyB(j,1) - xstar ) - atan2( xyB(j,2) - ystar , xyB(j,1) - xstar );
        theta = theta + deltatheta;
        velB(2,j) = deltatheta/SAMPLE_TIME; %w
        velB(1,j) = rstar * velB(2,j); %v     
    end
    
%     vxRef = [velA(1,:), velB(1,:)];   % Velocidad lineal a ser comandada
%     wRef = [velA(2,:), velB(2,:)];
%     v_cmd = vxRef(time_step-1);   % estas velocidades estan como ejemplo ...
%     w_cmd = wRef(time_step-1);    %      ... para que el robot haga algo.

  
        % fin del COMPLETAR ACA
    
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    if USE_ROOMBA       % para usar con el robot real
        
        % Enviar comando de velocidad
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometrÃ­a
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = ranges_full(1:DOWNSAMPLE_FACTOR:end);
        %ranges = circshift(ranges,length(ranges)/2);  % verificar
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        % Obtener pose del robot [x,y,yaw] de datos de odometrÃ­a (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,time_step) = [odompose.Pose.Pose.Position.X + INIT_POS(1); odompose.Pose.Pose.Position.Y+ INIT_POS(2); odomRotation(1)];
    
    else        % para usar el simulador
   
        % Mover el robot segun los comandos generados (ya vienen con ruido)
        [wL,wR] = inverseKinematics(diff_drive_obj,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(diff_drive_obj,wL,wR);
        vel_robot = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel_world = bodyToWorld(vel_robot,pose(:,time_step-1));  % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        pose(:,time_step) = pose(:,time_step-1) + vel_world*SAMPLE_TIME; 
        % Tomar nueva medicion del lidar
        ranges = lidar(pose(:,time_step));
        if SIMULATE_LIDAR_NOISE
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid=rand(length(ranges),1);
            ranges(not_valid<=chance_de_medicion_no_valida)=NaN;
        end
    end
    
    if time_step<LOCATION_ITERATION %Busco la posición incial
        particle_filter.predict(v_cmd,w_cmd,SAMPLE_TIME);
        if mod(time_step,PREDICTION_INTERVAL)==0
            particle_filter.correct(ranges,MAP,MAX_RANGE,"mse");
            particle_filter.Particles=generate_outliers(particle_filter,OUTLIERS_PCT,MAP,X_LIMS,Y_LIMS);
        end
    else
        display("Encontré mi posición")
        particle_filter.State %Este es el estado estimado
        particle_filter.predict(v_cmd,w_cmd,SAMPLE_TIME);
         if mod(time_step,PREDICTION_INTERVAL)==0
            particle_filter.correct(ranges,MAP,MAX_RANGE,"mse");
         end
    end
    
    %estimacion_estado=getStateEstimate(particle_filter)
    
    %%
    % Aca el robot ya ejecutÃ³ las velocidades comandadas y devuelve en la
    % variable ranges la medicion del lidar para ser usada y
    % en la variable pose(:,time_step) la odometrÃ­a actual.
    
    %% COMPLETAR ACA:
        % hacer algo con la medicion del lidar (ranges) y con el estado
        % actual de la odometria ( pose(:,time_step) )
        
        % Giro 360 e ir midiendo girando de a K° medimos y hacemos filtro de partículas.
            %Posicion inicial
            %Giramos 
            %Medimos
            %filtro de partículas
            %...
        % Tiramos A* para saber el camino (Ver de convolucionar mapa)
            % A* nos da X posiciones de las que necesitamos sólo N. 
            % Necesitamos N movimientos
            % Actualizamos filtro de partículas y A* cada N movimientos
            % (No medir si no actualizamos el filtro de particulas y actualizar con los N movimientos perdidos)
        % Llegamos y descansamos 3 segundos. 
        % Volver al inicio
        % Fin del COMPLETAR ACA
        
    %%
    % actualizar visualizacion
    markings=[WAYPOINTS;particle_filter.Particles(:,1:2)];
    visualizer(pose(:,time_step),markings,ranges)
    %WAYPOINTS=[1,1];
    waitfor(robot_sample_rate);
end

