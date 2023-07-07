load('traj\sysu_standard.mat','map');
map = ~map;
Imp_R = repmat(map*255, [1 1 3]);

% 加载路径数据
x_road = trajSYSU(1, :);
y_road = trajSYSU(2, :);
x_road = x_road * 100;
y_road = y_road * 100;
y_road = 600 - y_road;

% 加载轨迹数据（包含时间序列）
x_time = out.simout.X.Time; % x坐标数据
x = out.simout.X.Data;
y_time = out.simout.Y.Time; % y坐标数据
y = out.simout.Y.Data;

% 进行轨迹坐标系的转换
x = x * 100;
y = y * 100;
y = 600 - y;

% 创建一个新的图形窗口
figure;

% 显示地图
imshow(Imp_R);
hold on;

% 绘制绿色路径
plot(x_road, y_road, 'g', 'LineWidth', 2);

% 创建一个动态绘制轨迹的对象
trajLine = animatedline('Color', 'r', 'LineWidth', 2);

% 设置坐标轴标签
xlabel('X');
ylabel('Y');
title('Trajectory');

% 保持纵横比例
axis equal;

% 添加图例
legend('规划轨迹', '实际轨迹', 'Location', 'northwest', 'FontSize', 12);

% 初始化文本标签的位置
timeLabel = text(x(1), y(1), sprintf('Time: %.2f', x_time(1)), 'Color', 'r', 'FontSize', 12);

% 稀疏采样参数
sparseFactor = 15; % 每隔多少帧采样一次

% 创建一个存储帧的单元数组
frames = cell(ceil(length(x_time) / sparseFactor), 1);
frameIndex = 1;

% 逐步绘制轨迹并保存帧
for i = 1:sparseFactor:length(x_time)
    % 维护路径信息
    % 添加当前时间步骤的轨迹点
    addpoints(trajLine, x(i), y(i));
    
    % 更新文本标签的位置和内容
    set(timeLabel, 'Position', [x(i) y(i)], 'String', sprintf('Time: %.2f', x_time(i)));
    
    % 更新图形
    drawnow;
    
    % 将当前帧保存到单元数组中
    frames{frameIndex} = getframe(gcf);
    frameIndex = frameIndex + 1;
end

% 创建一个GIF文件并将帧写入其中
filename = 'trajectory.gif';
for i = 1:length(frames)
    frame = frames{i};
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % 在第一次迭代时创建GIF文件，之后追加帧
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
end

% 关闭绘图窗口
hold off;

