%% 加载.mat文件
load('sysu_standard.mat', 'map'); 
map = ~map;
MAX_X=size(map,1);
MAX_Y=size(map,2);

distanceFcn = @(p1,p2) norm(p1-p2);

%% AStar
GlbTab      = zeros(MAX_X, MAX_Y);  % 0|new 1|open 2|close
PathTab     = zeros(MAX_X, MAX_Y, 2);
nodeStartXY  = [530, 70];
nodeTargetXY = [56, 1125];  
startGn     = 0;
startHn     = distanceFcn(nodeTargetXY,nodeStartXY);
startFn     = startGn + startHn;
nodeStart  = [startFn, startGn, startHn, nodeStartXY]; 

%% loop
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
    
    for k= 1:-1:-1
        for j= 1:-1:-1
            if (k~=j || k~=0)  %The node itself is not its successor
                s_x  = node_x+k;
                s_y  = node_y+j;
                if((s_x >0 && s_x <=MAX_X) && (s_y >0 && s_y <=MAX_Y))%node within array bound
                    % exist close node
                    if GlbTab(s_x, s_y) == 2 || map(s_x,s_y) == 0 
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

%% get path
if foundpath == 1
    node_x = nodeTargetXY;
    Imp_R = repmat(map*255, [1 1 3]);
    lineWidth = 1; % Set the width of the line
    while ~isequal(node_x, nodeStartXY)
        for dx = -lineWidth:lineWidth
            for dy = -lineWidth:lineWidth
                x = node_x(1)+dx;
                y = node_x(2)+dy;
                % Check if the point is inside the image boundaries
                if (x >= 1) && (x <= size(map,1)) && (y >= 1) && (y <= size(map,2))
                    Imp_R(x,y,1) = 0;
                    Imp_R(x,y,2) = 255;
                    Imp_R(x,y,3) = 0;
                end
            end
        end
        node_x = PathTab(node_x(1),node_x(2),:);
        node_x = node_x(:)';
    end
    figure(1);
    imshow(Imp_R)
else
    disp('Not find solution')
end