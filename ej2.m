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
%% Parametros de la Simulacion
SIMULATION_DURATION = 3*60;          % Duracion total [s]
INIT_POS =[2.5; 1.5; -pi/2]; %random_empty_point(map);%        % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)

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


% Inicializar vectores de tiempo, entrada y pose
time_vec = 0:const.sample_time:SIMULATION_DURATION;     % Vector de Tiempo para duracion total
LOCATION_END = int32(2/const.sample_time);             %Iteraciones hasta ubicarse
pose = zeros(3,numel(time_vec));                        % Inicializar matriz de pose
pose(:,1) = INIT_POS;

%% Simulacion
robot_sample_rate = robotics.Rate(1/const.sample_time); %Para Matlab R2018b e inferiores

%Inicializo filtro de part�cula
x_lims=map.XWorldLimits;
y_lims=map.YWorldLimits;
POSITION_LIMITS = [x_lims(2),x_lims(1);y_lims(2),x_lims(1);pi,-pi];

visualizer = Visualizer2D();
visualizer.hasWaypoints=true;
visualizer.mapName = 'map';
attachLidarSensor(visualizer,lidar);
release(visualizer);
%%
%Genero comandos para localizarse
       % Velocidad angular a ser comandada

v_ref = zeros(1);
w_ref = zeros(1);

MAP_RES=25;             %Resoluci�n del mapa [celdas/metro]  
slam_obj=robotics.LidarSLAM(MAP_RES,const.lidar_max_range);
slam_obj.LoopClosureThreshold = 400;
slam_obj.LoopClosureSearchRadius = 4;
slam_obj.MovementThreshold=[0.5,0.5];
state="check path";
est_map=[];
est_pose=[0,0,0];
orientation_angle=0;
normal=[0,0];
for time_step = 2:length(time_vec) % Itera sobre todo el tiempo de simulaci�n
    
   %if length(w_ref) >time_step
    v_cmd = v_ref(time_step-1);   
    w_cmd = w_ref(time_step-1);
    
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
        ranges = ranges_full(1:const.lidar_downsample_factor:end);
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
    ranges(ranges<0.2)=NaN;
    min_distance=distance_to_obstacle(ranges);
    
    if length(w_ref)<=time_step-1
        %% rot angle
        %% orientation error
        
        path_blocked=true;
        if min_distance>const.obstacle_threshold 
            path_blocked=false;
        end
        [orientation_angle,normal]=calculate_orientation(ranges,path_blocked);
        if ~path_blocked %&& orientation_error<const.orientation_error_threshold
            state="move foward";
        else
            %orientation_angle=deg2rad(45);%abs(angdiff(est_pose(end,3),deg2rad(90)));
            state="rotate";
        end
    end
    if state=="move foward"
       state
       distance=const.distance_safety_factor*min_distance;
       speed_cmd=move_foward_command(distance);
       v_ref = [v_ref;speed_cmd(:,1)];
       w_ref = [w_ref;speed_cmd(:,2)];
       state="execute command";
    elseif state=="rotate" 
       state
       %quarter_point=int32(length(ranges)/4);
       %[max_val,max_index]=max(ranges(1:quarter_point));
       %target_angle=const.lidar_angle_start+(const.lidar_angle_end-const.lidar_angle_start)*max_index/length(ranges);
       rotation_angle=angdiff(orientation_angle,est_pose(end,3));
       speed_cmd=rotate_command(rotation_angle);
       v_ref = [v_ref;speed_cmd(:,1)];
       w_ref = [w_ref;speed_cmd(:,2)];
       state="execute command";
    elseif state=="execute command"
    end
    
    figure(1)
    if time_step==2 ||mod(time_step,10)==0
        %Ver de poner en una funci�n sola
        
        ranges=process_measurement(slam_obj,ranges);
        [scans_slam,est_pose] = scansAndPoses(slam_obj);
        est_map = buildMap(scans_slam,est_pose,MAP_RES,const.lidar_max_range);
        
        show(est_map);
        hold on;
        scatter(est_pose(end,1),est_pose(end,2),'filled','LineWidth',2);
        plot(est_pose(:,1),est_pose(:,2),'LineWidth',2);
        quiver(est_pose(end,1),est_pose(end,2),cos(est_pose(end,3)),sin(est_pose(end,3)),0.5,'filled','LineWidth',2);
        
        %quiver(intercept*cos(est_pose(end,3)),intercept*cos(est_pose(end,3)),normal(1),normal(2),0.5,'filled','LineWidth',2);
        
        axis([-3 3 -2.5 2.5])
        hold off;
        
    
    else
        intercept=ranges(int32(length(ranges)/2));
        markings=[pose(1,time_step)+intercept*cos(pose(3,time_step))+normal(1),pose(2,time_step)+intercept*sin(pose(3,time_step))+normal(2)];
        markings=[markings;pose(1,time_step)+intercept*cos(pose(3,time_step)),pose(2,time_step)+intercept*sin(pose(3,time_step))];
        visualizer(pose(:,time_step),markings,ranges)
    end
        waitfor(robot_sample_rate);
end
