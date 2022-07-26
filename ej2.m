%% Robot diferencial con lidar
% Robotica Movil - 2022 1c
% Funciona con MATLAB R2018a
close all
clear all
clc 

SIMULATE_LIDAR_NOISE = false; %simula datos no validos del lidar real, probar si se la banca
USE_ROOMBA = false;  % false para desarrollar usando el simulador, true para conectarse al robot real

%% Roomba
if USE_ROOMBA   % si se usa el robot real, se inicializa la conexion    
    init_connection()
end
    

%Objeto con constantes
const=Constants;
% creacion del Simulador de robot diferencial
diff_drive_obj = DifferentialDrive(const.wheel_separation,const.wheel_separation); 

%% Creacion del entorno
MAP_IMG = 1-double(imread('mapa_2022_1c.tiff'))/255;
map = robotics.OccupancyGrid(MAP_IMG, 25);

%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = const.lidar_offset;   
NUM_SCANS = 513/const.lidar_downsample_factor;
lidar.scanAngles = linspace(const.lidar_angle_start,const.lidar_angle_end,NUM_SCANS);
lidar.maxRange = const.lidar_max_range;

%% Crear visualizacion
visualizer = Visualizer2D();
visualizer.hasWaypoints=true;
visualizer.mapName = 'map';
attachLidarSensor(visualizer,lidar);
release(visualizer);

%% Parametros de la Simulacion
SIMULATION_DURATION = 3*60;          % Duracion total [s]
INIT_POS =random_empty_point(map,[5,1],[5,1]);%[2.5; 1.5; -pi/2];         % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)

% Inicializar vectores de tiempo, entrada y pose
time_vec = 0:const.sample_time:SIMULATION_DURATION;     % Vector de Tiempo para duracion total
LOCATION_END = int32(20/const.sample_time);             %Iteraciones hasta ubicarse
pose = zeros(3,numel(time_vec));                        % Inicializar matriz de pose
pose(:,1) = INIT_POS;

%% Simulacion
robot_sample_rate = robotics.Rate(1/const.sample_time); %Para Matlab R2018b e inferiores

%Inicializo filtro de partícula
x_lims=map.XWorldLimits;
y_lims=map.YWorldLimits;
POSITION_LIMITS = [x_lims(2),x_lims(1);y_lims(2),x_lims(1);pi,-pi];
X_LIMS = x_lims;
Y_LIMS = y_lims;


particle_filter=create_particle_filter();
initialize(particle_filter,const.particle_number,POSITION_LIMITS)

%%
%Genero comandos para localizarse
       % Velocidad angular a ser comandada

v_ref = zeros(LOCATION_END,1);
w_ref = const.angular_speed*ones(LOCATION_END,1);

for time_step = 2:length(time_vec) % Itera sobre todo el tiempo de simulación
    
   
    
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
        ranges = ranges_full(1:const.lidar_downsample_factor:end);
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
        pose(:,time_step) = pose(:,time_step-1) + vel_world*const.sample_time; 
        % Tomar nueva medicion del lidar
        ranges = lidar(pose(:,time_step));
        if SIMULATE_LIDAR_NOISE
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid = rand(length(ranges),1);
            ranges(not_valid <= chance_de_medicion_no_valida) = NaN;
        end
    end
    
    %markings=[WAYPOINTS;particle_filter.State(1:2)];
    visualizer(pose(:,time_step),markings,ranges)
    waitfor(robot_sample_rate);
end
