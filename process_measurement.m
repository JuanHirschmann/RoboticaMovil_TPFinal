function process_measurement(slam_obj,ranges)
     const=Constants;
     angles=linspace(const.lidar_angle_start,const.lidar_angle_end,length(ranges));
     scan=lidarScan(ranges,angles);
     addScan(slam_obj,scan);
end