clear all; clc;

N = 100;  
sigma = 5;  
noise_std = 9;  

image_dir = 'C:\Users\Άννα\Desktop\ΙΟΡΔΑΝΙΔΟΥ ΑΝΝΑ_1103984\Coke';
image_files = dir(fullfile(image_dir, '*.jpg'));  % Change this to the correct file type

output_file = 'or_tracking.avi';
video_writer = VideoWriter(output_file);
open(video_writer);

% Initialize particles
x0 = 324;
y0 = 198; 
particles = repmat([x0; y0], 1, N) + noise_std*randn(2, N);
weights = ones(1, N)/N;

for img_idx = 1:numel(image_files)
    frame = imread(fullfile(image_dir, image_files(img_idx).name));
    edges = edge(im2gray(frame), 'Canny');
   
    distances = bwdist(edges);
    likelihoods = exp(-distances.^2/(2*sigma^2));

    if sum(likelihoods(:))==0
        break;
    end

    particles = round(min(max(1, particles), [size(frame, 2); size(frame, 1)]));
    weights = likelihoods(sub2ind(size(frame), particles(2,:), particles(1,:)));
    weights = weights./sum(weights);

    % Resampling particles with the new weights
    resampled_particles = zeros(2, N);
    for i = 1:N
        idx = randsample(N, 1, true, weights);
        resampled_particles(:,i) = particles(:,idx);
    end
    particles = resampled_particles + noise_std*randn(2, N);
    estimated_location = mean(resampled_particles, 2);
    
    % Draw bounding box around estimated location
    bbox_x = max(1, round(estimated_location(1)-20));
    bbox_y = max(1, round(estimated_location(2)-20));
    bbox_w = min(size(frame,2)-bbox_x, 40);
    bbox_h = min(size(frame,1)-bbox_y, 40);
    
    [rows, cols] = find(edges(bbox_y:bbox_y+bbox_h, bbox_x:bbox_x+bbox_w));
    rows = rows + bbox_y - 1;  
    cols = cols + bbox_x - 1;

    centroid = [mean(cols), mean(rows)];  

    Mxx = mean((cols - centroid(1)).^2);
    Mxy = mean((cols - centroid(1)).*(rows - centroid(2)));
    Myy = mean((rows - centroid(2)).^2);

    estimated_orientation = 0.5 * atan2(2*Mxy, Mxx - Myy);
    
    corners = [bbox_x, bbox_y; 
               bbox_x + bbox_w, bbox_y; 
               bbox_x + bbox_w, bbox_y + bbox_h; 
               bbox_x, bbox_y + bbox_h]';

    bbox_centroid = [bbox_x + bbox_w/2; bbox_y + bbox_h/2];

    translated_corners = corners - bbox_centroid;

    rotation_matrix = [cos(estimated_orientation), -sin(estimated_orientation);
                       sin(estimated_orientation), cos(estimated_orientation)];

    rotated_corners = rotation_matrix * translated_corners;

    final_corners = rotated_corners + bbox_centroid;

    % Draw bounding box around estimated location
    bbox = [bbox_x, bbox_y, bbox_w, bbox_h];
    frame_with_bbox = insertShape(frame, 'Rectangle', bbox, 'LineWidth', 2);

    for i = 1:4
        frame_with_bbox = insertShape(frame_with_bbox, 'Line', [final_corners(:,i)', final_corners(:,mod(i,4)+1)'], 'LineWidth', 2, 'Color', 'green');
    end

    % Write out frame with bounding box to output video
    writeVideo(video_writer, frame_with_bbox);
end

% Close video writer
close(video_writer);
