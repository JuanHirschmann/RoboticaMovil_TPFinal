 classdef Constants
    properties( Constant = true )
        % Constantes del robot
         wheel_radius = 0.072/3;            %Radio de las ruedas [mts]
         wheel_separation = 0.235;          %Distancia entre ruedas [mts] 
         lidar_offset=[0,0];                %Offset del LiDAR [mts]
         sample_time=0.1;                   %Per�odo de muestreo del robot [s]
        % Constantes del LiDAR 
         lidar_downsample_factor=5;         %Factor de submuestreo de los angulos del LiDAR
         lidar_angle_start=deg2rad(-90);    %Angulo donde inicia el barrido[rad]
         lidar_angle_end=deg2rad(90);      %Angulo donde finaliza el barrido[rad]
         lidar_max_range=10;                %Rango m�ximo del LiDAR [mts]
        
        %Constantes de movimiento 
         angular_speed=0.25;                %Velocidad angular [rad/s]
         linear_speed=0.075;                  %Velocidad linear [mts/s]
         
        %Constantes de tiempos
         location_end_time=20;                           %Tiempo para finalizar la localizaci�n [s]
         %location_end_iteration=int32(20/sample_time);   %Iteraci�n donde finaliz� la localizaci�n [iteraciones]
         
        %Filtro de part�culas
         particle_number=1000;                   % Cantidad de particulas [particulas]
         particle_filter_resampling_interval=1;  % Pasos de correccion a realizar antes de remuestrear
         correction_interval=25;                 % Cantidad de pasos de predicci�n (movimientos) hasta realizar una correccion
         outliers_pct=0.05;                     % Porcentaje de part�culas que se generan aletoriamente
         
        %Modelo de medicion
         gaussian_model_sigma=0.1;               %Desv�o est�ndar del modelo gaussiano
         number_of_samples=10;                   %Cantidad de ang�los a analizar
        %Modelo de movimiento
                                  %Par�metros del modelo de movimiento
         alpha_1=0.05;
         alpha_2=0.1;
         alpha_3=0.1;
         alpha_4=0.25;
         alpha_5=1;
         alpha_6=1;
         minimum_angular_speed=0.000001;        %M�nima velocidad angular posible
         
        %Heur�stica 
         weight=7.5;
        %edge_cost
         threshold=0.1;
        
    end
 end