function [orientation_angle,wall_normal_vector]=calculate_orientation(ranges,blocked_path)
    const=Constants;
    %first_sample_index=1;
    %first_sample_angle=const.lidar_angle_start;
    lidar_angle_delta=(const.lidar_angle_end-const.lidar_angle_start)/length(ranges);
    %if blocked_path
    first_sample_index=int32(length(ranges)/2-const.samples_angle/(2*lidar_angle_delta));
    first_sample_angle=-const.samples_angle/2;
    %end
    last_sample_index=first_sample_index+int32(const.samples_angle/(lidar_angle_delta));
    last_sample_angle=first_sample_angle+const.samples_angle;
    [first_sample_x,first_sample_y]=pol2cart(first_sample_angle,ranges(first_sample_index));
    [last_sample_x,last_sample_y]=pol2cart(last_sample_angle,ranges(last_sample_index));
    first_sample=[first_sample_x,first_sample_y];
    last_sample=[last_sample_x,last_sample_y];
    
    %orientation_angle=atan2(first_sample(2)-last_sample(2),first_sample(1)-last_sample(1));
    orientation_angle=0;
    wall_normal_vector=find_normal_vector(first_sample,last_sample);
    measurement_vector=[ranges(int32(length(ranges)/2)),0];
    orientation_angle=atan2(wall_normal_vector(2),wall_normal_vector(2));
    if abs(orientation_angle)>pi/2
        orientation_angle=orientation_angle-pi;
    end
end