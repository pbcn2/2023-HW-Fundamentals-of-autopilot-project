clc; clear;

% 加载.mat文件
load('sysu_standard.mat', 'map'); 
map = ~map;
MAX_X=size(map,1);
MAX_Y=size(map,2);

% 使用D*算法进行路径规划
planner = plannerDStar(map, 'MinIterations', 100);

% 设置起始点和目标点
start = [520, 80];
goal = [150, 1130]; 

% 计划路径
refpath = plan(planner, start, goal);

% 显示地图和路径
show(planner)

% 如果需要调整地图，可以调用updateOccupancy，并再次规划
% 修改地图
% map = map';
% updateOccupancy(planner, map);
% refpath = plan(planner, start, goal);
% show(planner)
