load('sysu_standard.mat');

% 1表示障碍物，0表示可通行的空间。
mapData = map; 

% 创建一个占用格对象
map_new = binaryOccupancyMap(mapData);

% 定义起始和目标位置
start = [520 80];
goal = [150 1130];

% 创建A*规划器
planner = plannerAStarGrid(map_new);

% 计算路径
[route, numExpanded] = plan(planner, start, goal);

% 将路径分解为X和Y组件
routeX = route(:,1);
routeY = route(:,2);

% 通过使用样条插值生成平滑的路径。参数'p'可以更改以调整平滑程度。
p = 0.7; % p值的范围是0到1，越接近1，路径越平滑。
routeXSmooth = csaps(1:numel(routeX), routeX, p, 1:numel(routeX));
routeYSmooth = csaps(1:numel(routeY), routeY, p, 1:numel(routeY));

% 显示结果
figure
show(map_new)
hold on
plot(routeXSmooth, routeYSmooth, 'r', 'LineWidth', 2) % 显示平滑后的路径
hold off
