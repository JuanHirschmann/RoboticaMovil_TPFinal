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
load 2021_2c_tp_map.mat     %carga el mapa como occupancyMap en la variable 'MAP'

%if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
MAP_IMG = 1-double(imread('imagen_2021_2c_mapa_tp.tiff'))/255;
MAP = robotics.OccupancyGrid(MAP_IMG, 25);
%elseif verMatlab.Release(1:5)=='(R2018a)'    % Completar con la version que tengan
    %Ni idea que pasa, ver si el truco R2016b funciona
    %disp('ver si la compatibilidad R2016b funciona');
%else
    %disp(['Utilizando MATLAB ', verMatlab.Release]);
%end

%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = [0,0];   % Posicion del sensor en el robot (asumiendo mundo 2D)
DOWNSAMPLE_FACTOR = 5;                %decimar lecturas de lidar acelera el algoritmo
NUM_SCANS = 513/DOWNSAMPLE_FACTOR;
LIDAR_ANGLE_START = deg2rad(-90);
LIDAR_ANGLE_END = deg2rad(90);

lidar.scanAngles = linspace(LIDAR_ANGLE_START,LIDAR_ANGLE_END,NUM_SCANS);
lidar.maxRange = 10;

%% Crear visualizacion
visualizer = Visualizer2D;
visualizer.mapName = 'MAP';
attachLidarSensor(visualizer,lidar);

%% Parametros de la Simulacion

SIMULATION_DURATION = 3*60;          % Duracion total [s]
SAMPLE_TIME = 0.1;                   % Sample time [s]
INIT_POS = [2; 2.5; -pi/2];         % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)

% Inicializar vectores de tiempo, entrada y pose
time_vec = 0:SAMPLE_TIME:SIMULATION_DURATION;         % Vector de Tiempo para duracion total

%% generar comandos a modo de ejemplo
vxRef = 0.05*ones(size(time_vec));   % Velocidad lineal a ser comandada
wRef = zeros(size(time_vec));       % Velocidad angular a ser comandada
wRef(time_vec < 5) = -0.2;
wRef(time_vec >=7.5) = 0.2;

pose = zeros(3,numel(time_vec));    % Inicializar matriz de pose
pose(:,1) = INIT_POS;

%% Simulacion

robot_sample_rate = robotics.Rate(1/SAMPLE_TIME); %Para Matlab R2018b e inferiores

PARTICLES_NUM=100;
particles=initialize_particles(PARTICLES_NUM);

for time_step = 2:length(time_vec) % Itera sobre todo el tiempo de simulaci�n

    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.   
    v_cmd = vxRef(time_step-1);   % estas velocidades estan como ejemplo ...
    w_cmd = wRef(time_step-1);    %      ... para que el robot haga algo.
    
    %% COMPLETAR ACA:
        % generar velocidades para este timestep
        
        % fin del COMPLETAR ACA
    
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    
    if USE_ROOMBA       % para usar con el robot real
        
        % Enviar comando de velocidad
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometría
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = ranges_full(1:DOWNSAMPLE_FACTOR:end);
        %ranges = circshift(ranges,length(ranges)/2);  % verificar
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        % Obtener pose del robot [x,y,yaw] de datos de odometría (integrado por encoders).
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
    %%
    % Aca el robot ya ejecutó las velocidades comandadas y devuelve en la
    % variable ranges la medicion del lidar para ser usada y
    % en la variable pose(:,time_step) la odometría actual.
    
    %% COMPLETAR ACA:
        % hacer algo con la medicion del lidar (ranges) y con el estado
        % actual de la odometria ( pose(:,time_step) )
        
        
        % Fin del COMPLETAR ACA
        
    %%
    % actualizar visualizacion
    visualizer(pose(:,time_step),ranges)
    waitfor(robot_sample_rate);
end
