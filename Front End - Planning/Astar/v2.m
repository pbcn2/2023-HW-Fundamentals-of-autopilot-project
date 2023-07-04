clc; clear;

% ===================================
% verson 2.0
% 这个版本的代码实现了更加稀疏的点，便于插值
% ===================================

%% Load the .mat file
load('sysu_standard.mat'); 
MAX_X=size(map,1);
MAX_Y=size(map,2);

%% Create a color map
colorMap = zeros(MAX_X, MAX_Y, 3);  % Initialize to black

% Red channel = obstacles
% Green channel = free space
% You can adjust these values to change the colors
obstacleColor = [0.95, 0.5, 0.7]; % 粉红色
freeSpaceColor = [0.6, 0.9, 0.8]; % 蓝绿色
inflatedObstacleColor = [0.75, 0.6, 0.95]; % 紫色

% Apply the colors to the map
colorMap(:,:,1) = map * obstacleColor(1) + ~map * freeSpaceColor(1);
colorMap(:,:,2) = map * obstacleColor(2) + ~map * freeSpaceColor(2);
colorMap(:,:,3) = map * obstacleColor(3) + ~map * freeSpaceColor(3);

%% Obstacle Inflation
r = 9;  % Set the inflation radius
se = strel('square', 2*r+1);  % Create a square structural element
inflatedMap = imdilate(map, se);  % Dilate the obstacles

% Mark the inflated obstacles in the color map
colorMap(:,:,1) = colorMap(:,:,1) + inflatedObstacleColor(1) * (inflatedMap & ~map);
colorMap(:,:,2) = colorMap(:,:,2) + inflatedObstacleColor(2) * (inflatedMap & ~map);
colorMap(:,:,3) = colorMap(:,:,3) + inflatedObstacleColor(3) * (inflatedMap & ~map);

% Now invert the map again for the path planning
map = ~inflatedMap;

distanceFcn = @(p1,p2) norm(p1-p2);

%% AStar
GlbTab      = zeros(MAX_X, MAX_Y);  % 0|new 1|open 2|close
PathTab     = zeros(MAX_X, MAX_Y, 2);
nodeStartXY  = [520, 80];
nodeTargetXY = [150, 1130];  
startGn     = 0;
startHn     = distanceFcn(nodeTargetXY,nodeStartXY);
startFn     = startGn + startHn;
nodeStart  = [startFn, startGn, startHn, nodeStartXY]; 

%% Loop
step_big = 10;  % Big step size
step_small = 1;  % Small step size

openset = [nodeStart];
foundpath = 0;
while(~isempty(openset))
    [~,minIdx] = min(openset(:,1));
    node = openset(minIdx,:);
    openset(minIdx,:) = [];
    if isequal(node(4:5), nodeTargetXY) 
       foundpath = 1;
       break;
    end
    node_x  = node(4);
    node_y  = node(5);
    node_gn = node(2);
    GlbTab(node_x, node_y) = 2;
    
    for k= -step_big:step_big:step_big
        for j= -step_big:step_big:step_big
            if (k~=j || k~=0)  %The node itself is not its successor
                s_x  = node_x+k;
                s_y  = node_y+j;
                if((s_x >0 && s_x <=MAX_X) && (s_y >0 && s_y <=MAX_Y))%node within array bound
                    % exist close node
                    if GlbTab(s_x, s_y) == 2 || map(s_x,s_y) == 0 
                        % Go back to fine search if it's an obstacle
                        for m= -step_small:step_small:step_small
                            for n= -step_small:step_small:step_small
                                s_x  = node_x+m;
                                s_y  = node_y+n;
                            end
                        end
                        continue;
                    end
                    s_gn = node_gn + distanceFcn([node_x,node_y], [s_x,s_y]);
                    s_hn = distanceFcn(nodeTargetXY, [s_x,s_y]);
                    s_fn = s_gn + s_hn;
                    if GlbTab(s_x, s_y) == 0
                        % new node
                        GlbTab(s_x, s_y) = 1;
                        openset = [openset; [s_fn, s_gn, s_hn, s_x, s_y]];
                        PathTab(s_x, s_y, :) = [node_x, node_y];
                    else
                        % exist open node
                        x_set = openset(:,4);
                        y_set = openset(:,5);
                        existIdx = find(x_set == s_x & y_set ==s_y, 1);
                        assert(~isempty(existIdx))
                        exist_gn = openset(existIdx, 2);
                        if exist_gn > s_gn
                            openset(existIdx,:) = [s_fn, s_gn, s_hn, s_x, s_y];
                            PathTab(s_x, s_y, :) = [node_x, node_y];
                        end
                    end
                end
            end
        end
    end
end

%% Get path
if foundpath == 1
    node_x = nodeTargetXY;
    lineWidth = 1; % Set the width of the line
    path = zeros(4, 0);  % Initialize path array
    
    while ~isequal(node_x, nodeStartXY)
        for dx = -lineWidth:lineWidth
            for dy = -lineWidth:lineWidth
                x = node_x(1)+dx;
                y = node_x(2)+dy;
                % Check if the point is inside the image boundaries
                if (x >= 1) && (x <= size(map,1)) && (y >= 1) && (y <= size(map,2))
                    colorMap(x,y,1) = 255;
                    colorMap(x,y,2) = 0;
                    colorMap(x,y,3) = 0;
                end
            end
        end
        psi = compute_psi(PathTab, node_x);  % You need to implement this function
        curvature = compute_curvature(PathTab, node_x);  % You need to implement this function
        path = [[MAX_X - node_x(1); node_x(2); psi; curvature], path];  % Store path information at the beginning of the matrix
        
        node_x = squeeze(PathTab(node_x(1), node_x(2), :))';
    end
    
        % Scale coordinates
    path(1:2, :) = path(1:2, :) * 0.01;  % Scale coordinates
    
    % Swap x and y coordinates
    temp1 = path(1, :);
    path(1, :) = path(2, :);
    path(2, :) = temp1;
    
    % Adjust psi and curvature after swapping x and y coordinates
    % Convert psi from y-axis origin to x-axis origin by subtracting pi/2
    path(3, :) = path(3, :) - pi/2;
    path(3, :) = atan2(sin(path(3, :)), cos(path(3, :)));  % Normalize psi to [-pi, pi]
%     % Assuming curvature is positive when turning left, we negate it after swapping x and y
%     path(4, :) = -path(4, :);

% =============================================

% =============================================

    % Save path to file
    trajSYSU = path;
    save('traj_diySYSU.mat', 'trajSYSU');
    
    figure(1);
    imshow(colorMap)
else
    disp('Not find solution')
end