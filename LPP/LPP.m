% Local Path Planner using IIR Low Pass Filter
% Written by Hyosung Hong (2017.04.15)
clear; close all; clc;
path = load('car_sim_path_hong.txt');
WP_size = 40;
WP_distance_gap = sqrt((path(2,1)-path(1,1))^2+(path(2,2)-path(1,2))^2);
for init_indx=1:size(path,1)-WP_size
    local_offset = [0,randn];   % Local y 축 (lateral) 방향으로 offset 되어 있는 거리
    original_path = path(init_indx:init_indx+WP_size-1,1:2);
    local_path_filtered = original_path;
    local_path = original_path;
    
    init_heading = atan2(original_path(2,2)-original_path(1,2), original_path(2,1)-original_path(1,1));
    local_path_filtered(1,:) = [cos(init_heading), -sin(init_heading); sin(init_heading), cos(init_heading)]*local_offset' + original_path(1,:)';
    init_path_gap = local_path_filtered(1,:) - original_path(1,:);
    %     local_path_filtered(2,:) = [cos(init_heading), -sin(init_heading); sin(init_heading), cos(init_heading)]*local_offset' + original_path(2,:)';
    local_path_filtered(2,:) = original_path(2,:) + init_path_gap;
    local_path(1:2,:) = local_path_filtered(1:2,:);

    WP_index = 3;
    for i=3:WP_size
%         local_path_filtered(i,:) = 0.0020775900*original_path(i,:) + 0.0041551801*original_path(i-1,:) + 0.0020775900*original_path(i-2,:) + 1.8669904265*local_path_filtered(i-1,:) + -0.8753007866*local_path_filtered(i-2,:);    % 15 Hz
%         local_path_filtered(i,:) = 0.0036125749*original_path(i,:) + 0.0072251499*original_path(i-1,:) + 0.0036125749*original_path(i-2,:) + 1.8229266924*local_path_filtered(i-1,:) + -0.8373769921*local_path_filtered(i-2,:);    % 20 Hz
        local_path_filtered(i,:) = 0.0077769953*original_path(i,:) + 0.0155539906*original_path(i-1,:) + 0.0077769953*original_path(i-2,:) + 1.7355001605*local_path_filtered(i-1,:) + -0.7666081417*local_path_filtered(i-2,:);    % 30 Hz
    end
    
    i2 = 2;
    for i=2:WP_size
                
        WP_gap = sqrt((original_path(i2,1)-original_path(i2-1,1))^2 + (original_path(i2,2)-original_path(i2-1,2))^2);
        position_ratio = (((local_path_filtered(i,1)-original_path(i2-1,1))*(original_path(i2,1)-original_path(i2-1,1)))+((local_path_filtered(i,2)-original_path(i2-1,2))*(original_path(i2,2)-original_path(i2-1,2))))/WP_gap^2;
        x_d = original_path(i2-1,1)+position_ratio*(original_path(i2,1)-original_path(i2-1,1));    % desired x position (perpendicular point from the path)
        y_d = original_path(i2-1,2)+position_ratio*(original_path(i2,2)-original_path(i2-1,2));    % desired y position (perpendicular point from the path)
        e_l=sqrt((x_d-local_path_filtered(i,1))^2+(y_d-local_path_filtered(i,2))^2);
        if e_l < 0.05
            break
        end
        while position_ratio > 1
            i2 = i2 + 1;
            WP_gap = sqrt((original_path(i2,1)-original_path(i2-1,1))^2 + (original_path(i2,2)-original_path(i2-1,2))^2);
            position_ratio = (((local_path_filtered(i,1)-original_path(i2-1,1))*(original_path(i2,1)-original_path(i2-1,1)))+((local_path_filtered(i,2)-original_path(i2-1,2))*(original_path(i2,2)-original_path(i2-1,2))))/WP_gap^2;
        end
        local_path(i2,:) = local_path_filtered(i,:);
    end
    
    plot(original_path(:,1), original_path(:,2),'b.')
    hold on
    plot(original_path(1,1), original_path(1,2),'b*')
    plot(local_path(1,1), local_path(1,2),'rs')
    plot(local_path(:,1), local_path(:,2),'r.')
    axis equal
    hold off
    pause()
end