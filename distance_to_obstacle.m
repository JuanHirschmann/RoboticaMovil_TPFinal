function min_distance=distance_to_obstacle(ranges)
    mid_point=int32(length(ranges)/2);
    deviation=10;
    center_measurements=ranges(mid_point-deviation:mid_point+deviation);
    min_distance=min(center_measurements);
end