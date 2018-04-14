%% MoccGrid: Mockup of Occupancy Grid
% Mapping with known poses: Incorporate laser sensor readings into map

%% Setup
clear;
close all

% Create POG object
probMap = robotics.OccupancyGrid(16,16,20);
probMap.GridLocationInWorld = [-8 -8];
probMap.OccupiedThreshold = 0.6;
probMap.FreeThreshold = 0.45;
% figure; a.show; title('Occupancy Grid');

% Create BOG object
binMap = robotics.BinaryOccupancyGrid(16,16,20);
binMap.GridLocationInWorld = [-8 -8];
% figure; map.show; title('Binary Occupancy Grid');

% Create video writer
writerObj = VideoWriter('MoccGrid.avi','Uncompressed AVI');
writerObj.FrameRate = 15;

%% Import sensor readings and robot poses from a rosbag
% Create a |robotics.ros.BagSelection| object from a stored ROS bag file
% using the |rosbag()| function
filePath = fullfile(fileparts(which('MapUpdateUsingSensorDataExample')), 'data', 'rosbagWithSensorData.bag');
bag = rosbag(filePath);

% Extract the robot poses and laser range readings from a rosbag file. In
% this example, a helper file is used, |exampleHelperReadPoseAndSensorMsgs|.
[poses, laserRanges] = exampleHelperReadPoseAndSensorMsgs(bag);

%% Incorporating sensor reading into map
totalNCellsUpdated = 0;
tic;
open(writerObj); %open video writer
for j=1:length(poses)
    % Define the Quaternion in a vector form
    q = [poses{j}.Orientation.W, poses{j}.Orientation.X, poses{j}.Orientation.Y, poses{j}.Orientation.Z];

    % Obtain robot position in grid coordinates
    posn = [poses{j}.Position.X, poses{j}.Position.Y];
    posn = world2grid(probMap, posn);
    
    % Convert the Quaternion into Euler angles
    orientation = quat2eul(q, 'ZYX');
    
    % The z-direction rotation component is the heading of the
    % robot. Use the heading value along with the z-rotation axis to define
    % the rotation matrix
    rotMatrixAlongZ = axang2rotm([0 0 1 orientation(1)]);
    
    % Convert sensor values (originally in the robot's frame) into the
    % world coordinate frame. This is a 2D coordinate transformation, so
    % only the first two rows and columns are used from the rotation matrix.
    rangesInWorldFrame = rotMatrixAlongZ(1:2,1:2) * laserRanges{j}';
    
    % Update map based on laser scan data in the world coordinate frame.
    % Populate the map, using the setOccupancy function, by setting the
    % obstacle location on the map as occupied for every sensor reading that
    % detects obstacles
        setOccupancy(binMap, rangesInWorldFrame', 1);
    
    % Obtain vertices of polygon to fill with hits & misses
    vertices = world2grid(probMap, rangesInWorldFrame');
    xvertices = [posn(1); vertices(:,1)];
    yvertices = [posn(2); vertices(:,2)];

    % line1 and line2 are two lines of the above polygon containing the
    % corresponding to the first/leftmost and last/rightmost rays of the scanner. These lines
    % don't neede to add hits
    xline1 = [posn(1) vertices(1,1)];
    yline1 = [posn(2) vertices(1,2)];
    xline2 = [posn(1) vertices(end,1)];
    yline2 = [posn(2) vertices(end,2)];
    
    % Check free/occupied cells within bounding box of vertices
    i = min(xvertices):max(xvertices);
    j = min(yvertices):max(yvertices);
    [X, Y] = meshgrid(i, j); %2D grid from i and j
    %checking if it is in/on the polygon
    [in, on] = inpolygon(X,Y,xvertices,yvertices);

    % variables to check if points are on line1, line2
    on1 = false(size(on));
    on2 = false(size(on));
    v1 = vertices(1,:); %vpolygon ertex from first laser reading
    v2 = vertices(end,:); %polygon vertex from last laser reading

    for i = 1:length(X(:))
        %checking if dist from X,Y to line1 is 0 (i.e. on the line)
        temp = det([[X(i) Y(i)] - posn; v1-posn]);
        on1(i) = (temp == 0);
        
        %checking if dist from X,Y to line2 is 0 (i.e. on the line)
        temp = det([[X(i) Y(i)] - posn; v2-posn]);
        on2(i) = (temp == 0);
    end
    
    % update in to reflect points that are _strictly_ inside the polygon
    in = logical(in - on);
    % update on to reflect points that are on polygon (except for line1, line2)
    on = logical(on - (on1 | on2));
    
    % Get no of cells updated
    totalNCellsUpdated = totalNCellsUpdated + size(find(on),1) + size(find(in),1);
    
    % Add hits and misses accordingly
%     addHit(probMap,[X(on) Y(on)],'grid');
updateLogOdds(probMap, [X(on) Y(on)], probMap.OccupiedThreshold, 'grid');
%     addMiss(probMap,[X(in) Y(in)],'grid');
updateLogOdds(probMap, [X(in) Y(in)], probMap.FreeThreshold, 'grid');

    % Update frame for video writer
    probMap.show;
    frame = getframe;
    writeVideo(writerObj,frame);    
end
close(writerObj); %close video writer
time = toc;
disp(['Total number of cells updated = ' num2str(totalNCellsUpdated)]);
disp(['Number of cells updated per second = ' num2str(totalNCellsUpdated/time)]);

%% Show grids
% Show ternary map grid
cMap = colormap(flip(colormap('gray')));

terMap = probMap.applyOccupancyThresholds;
figure; imshow(terMap, [0 1]); 
title('Ternary Occupancy Grid'); colormap(cMap);

% Show probabilistic and binary maps
figure; probMap.show; title('Probabilistic Occupancy Grid');
figure; binMap.show; title('Binary Occupancy Grid');

% Show LogOdds
figure; imshow(probMap.LogOdds, []); title('POG LogOdds');
colormap(flip(colormap('gray')));

%% End

% clear;
% close all;