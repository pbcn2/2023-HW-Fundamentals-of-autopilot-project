import numpy as np
from scipy.ndimage import distance_transform_edt as bwdist
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
from skimage import graph
from scipy.io import loadmat

def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w

# 加载.mat文件
mat = loadmat('Front End - Planning\sysu_standard.mat')

# 获取map和out变量
mapData = mat['map']
out = mat['out']

# 对障碍物进行膨胀
mapData_bak = mapData  # 备份未膨胀的地图
mapData = binary_dilation(mapData, structure=np.ones((35, 35)))

# 垂直翻转地图
mapData_bak = np.flipud(mapData_bak)
mapData = np.flipud(mapData)

# 垂直翻转起点和终点的坐标
height = mapData.shape[0]
start = (height - 520, 80)
goal = (height - 150, 1130)

# 使用Scipy的A*算法找到路径
weights = 1.0 + 20.0 * bwdist(mapData == 1) ** 0.4
path, _ = graph.route_through_array(weights, start, goal, fully_connected=True)

# 将路径分解为X和Y组件
path = np.array(path).T
routeX, routeY = path[0], path[1]

# 设置滑动窗口的大小
window_size = 60

# 在路径的起始和结束处添加额外的点
padding_points = window_size // 2
padded_routeX = np.pad(routeX, (padding_points, padding_points), mode='edge')
padded_routeY = np.pad(routeY, (padding_points, padding_points), mode='edge')

# 计算移动平均
routeXSmooth = moving_average(padded_routeX, window_size)
routeYSmooth = moving_average(padded_routeY, window_size)

# 显示结果
plt.figure()
plt.imshow(mapData_bak, origin='lower', cmap='gray')
plt.plot(routeY, routeX, 'y.')  # 原始路径
plt.plot(routeYSmooth, routeXSmooth, 'r-', linewidth=2)  # 平滑后的路径
plt.plot(start[1], start[0], 'go')  # 起点
plt.plot(goal[1], goal[0], 'bo')  # 终点
plt.show()
