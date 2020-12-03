function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====

    % Write your code here...
    % useful constants
    m = size(fusion_map.pointcloud.Location,1); n = size(fusion_map.pointcloud.Location,2);
        
    K_in = [fx,0,0; 0, fy, 0; cx, cy, 1];
    pc_camera = pctransform(fusion_map.pointcloud, invert(tform));

    % fusion map qtys
%     points_ground = cat(1, reshape( permute(fusion_map.pointcloud.Location,[3 2 1]), 3,1, [] ), ones(1,1, m) ); 
    points_ground = reshape(fusion_map.pointcloud.Location', 3,1,[]);    
    normals_ground = reshape(fusion_map.normals', 3,1,[]);    
    ccount_ground = reshape(fusion_map.ccounts, 1,1,[]);
    times_ground = reshape(fusion_map.times, 1,1,[]);
    color_ground = reshape(fusion_map.pointcloud.Color', 3,1,m) ;

    % transform to camera frame
    points_camera = reshape(pc_camera.Location', 3,1,[]);    
    
    % find corresp pixel values
    pix_img = sum( repmat(K_in',[1 1 m]) .* permute(points_camera(1:3,:,:), [2 1 3]) , 2);
    pix_img = round( pix_img(1:2,:,:)./repmat(points_camera(3,:,:), [2 1 1]) );    
    
    % find FOV conditions
    fov_cond1 = pix_img(1,1,:)>0;
    fov_cond2 = pix_img(1,1,:)<w;
    fov_cond3 = pix_img(2,1,:)>0; 
    fov_cond4 = pix_img(2,1,:)<h;
    fov_cond5 = points_camera(3,1,:)>0;
    fov_tot = bsxfun( @and, fov_cond1, fov_cond2);
    fov_tot = bsxfun( @and, fov_tot, fov_cond3);
    fov_tot = bsxfun( @and, fov_tot, fov_cond4);
    fov_tot = bsxfun( @and, fov_tot, fov_cond5);
    fov_idxs = find(fov_tot==1)';    
    pix_fov = pix_img(:,:,fov_idxs);
    
    % find accepted points from fusion map based on FOV    
    points_cam_fov = points_ground(:,:,fov_idxs);
    color_cam_fov =  color_ground(:,:,fov_idxs);
    normals_cam_fov = normals_ground(:,:,fov_idxs);
    ccount_fov = ccount_ground(:,:,fov_idxs);
    times_fov = times_ground(:,:,fov_idxs);
    
    % start projecting into pixel space
    proj_points = zeros(h,w,3);
    proj_colors = zeros(h,w,3);
    proj_normals = zeros(h,w,3);
    proj_ccounts = zeros(h,w);
    proj_times = zeros(h,w);

    % figure out which pixels to assign
    pix_col = reshape(pix_fov(1,1,:), 1,[]);
    pix_row = reshape(pix_fov(2,1,:), 1,[]);
    L1 = [pix_row, pix_row, pix_row];
    L2 = [pix_col, pix_col, pix_col];
    L3 = [ones(1, size(pix_fov,3)), 2*ones(1, size(pix_fov,3)), 3*ones(1, size(pix_fov,3))] ;
    l_ind = sub2ind([h, w, 3] ,L1, L2, L3);
    R1 = [ones(1, size(pix_fov,3)), 2*ones(1, size(pix_fov,3)), 3*ones(1, size(pix_fov,3))] ;
    R2 = ones(1, 3*size(pix_fov,3));
    R3 = [1:size( points_cam_fov,3), 1:size( points_cam_fov,3), 1:size( points_cam_fov,3)] ;
    r_ind = sub2ind( [3,1,size( points_cam_fov,3)], R1, R2, R3 );
    
    % do the actual assignment
    proj_points( l_ind) =  points_cam_fov(r_ind);
    proj_colors(l_ind) = color_cam_fov(r_ind);
    proj_normals(l_ind) = normals_cam_fov(r_ind);
    
    l_ind1 = sub2ind( [h,w], pix_row, pix_col);
    r_ind1 = sub2ind( [1,1,size(pix_fov,3)], ones(1, size(pix_fov,3)), ones(1, size(pix_fov,3)), ...
        1:size( points_cam_fov,3) );
    proj_times(l_ind1) = times_fov(r_ind1);
    proj_ccounts(l_ind1) = ccount_fov(r_ind1);

    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
    proj_flag = fov_idxs;
end
