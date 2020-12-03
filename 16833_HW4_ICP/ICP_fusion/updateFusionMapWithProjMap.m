function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    % Write your code here...
    fov_idxs = proj_flag;    
    
    update_pts_flat = reshape( permute(updated_map.points,[3 2 1]), 3, h*w )' ;
    update_norm_flat = reshape( permute(updated_map.normals,[3 2 1]), 3, h*w )' ;
    update_col_flat = reshape( permute(updated_map.colors,[3 2 1]), 3, h*w )' ;
    update_time_flat = reshape( updated_map.times', 1, h*w )' ;
    update_cc_flat = reshape( updated_map.ccounts', 1, h*w )' ;

    num_old = size(fusion_map.pointcloud.Location,1)-size(fov_idxs,2);
    old_pts = fusion_map.pointcloud.Location;
    old_pts(fov_idxs, :) = [];
    old_norm = fusion_map.normals;
    old_norm(fov_idxs, :) = [];
    old_col = fusion_map.pointcloud.Color;
    old_col(fov_idxs, :) = [];
    old_times = fusion_map.times;
    old_times(fov_idxs, :) = [];
    old_cc = fusion_map.ccounts;
    old_cc(fov_idxs, :) = [];
    
    map_points = zeros( num_old +   h*w, 3 );
    map_points(1:num_old, :) = old_pts;
    map_points( num_old + 1:end, : ) = update_pts_flat;
    
    map_normals = zeros( num_old +   h*w, 3 );
    map_normals(1:num_old, :) = old_norm;
    map_normals( num_old + 1:end, : ) = update_norm_flat;
    
    map_colors = zeros( num_old +   h*w, 3 );
    map_colors(1:num_old, :) = old_col;
    map_colors( num_old + 1:end, : ) = update_col_flat;
    
    map_times = zeros( num_old +   h*w, 1 );
    map_times(1:num_old, :) = old_times;
    map_times( num_old + 1:end, : ) = update_time_flat;
    
    map_ccounts = zeros( num_old +   h*w, 1 );
    map_ccounts(1:num_old, :) = old_cc;
    map_ccounts( num_old + 1:end, : ) = update_cc_flat;
      
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', uint8(map_colors));
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   