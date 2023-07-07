import numpy as np
from scipy.ndimage import distance_transform_edt as bwdist
from scipy.ndimage import binary_dilation
from skimage import graph
from scipy.io import loadmat
from scipy.io import savemat
import math

def moving_average(x, w):
    # 滑动窗口平滑
    return np.convolve(x, np.ones(w), 'valid') / w


def calculate_curvature(x1, y1, x2, y2, x3, y3):
    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x3 - x2
    dy2 = y3 - y2

    if dx1 * dy2 - dy1 * dx2 == 0:
        return 0
    else:
        return ((dx1 * dy2) - (dy1 * dx2)) / ((dx1**2 + dy1**2) * math.sqrt((dx2**2 + dy2**2)))



# ============================================================================

# 加载.mat文件
mat = loadmat('sysu_standard.mat')

# 获取map和out变量
mapData = mat['map']
# mapData = np.array(mapData == 111, dtype=bool)
# mapData = np.asmatrix(mapData)
# out = mat['out']

# 对障碍物进行膨胀
mapData_bak = mapData  # 备份未膨胀的地图
mapData = binary_dilation(mapData, structure=np.ones((35, 35)))

# 垂直翻转地图
mapData_bak = np.flipud(mapData_bak)
mapData = np.flipud(mapData)

# 垂直翻转起点和终点的坐标
height = mapData.shape[0]
start = (height - 520, 72)
goal = (height - 140, 1140)

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

# 保存平滑路径到矩阵并缩小100倍
smooth_path = np.column_stack((routeYSmooth / 100, routeXSmooth / 100))
smooth_path_with_heading = np.zeros((smooth_path.shape[0]-1, 4))


for i in range(smooth_path.shape[0] - 1):  # 减去1以避免超出索引范围
    x = smooth_path[i, 0]
    y = smooth_path[i, 1]
    heading = math.atan2(smooth_path[i+1, 1] - y, smooth_path[i+1, 0] - x)
    curvature = 0.0  # 这里暂时设置为0，需要根据实际情况计算曲率
    smooth_path_with_heading[i] = [x, y, heading, curvature]

# 在smooth_path_with_heading中添加曲率
# 在smooth_path_with_heading中添加曲率
for i in range(1, len(smooth_path)-1):  # 跳过第一个和最后一个点，因为它们没有足够的邻居来计算曲率
    curvature = calculate_curvature(smooth_path[i-1, 0], smooth_path[i-1, 1],
                                    smooth_path[i, 0], smooth_path[i, 1],
                                    smooth_path[i+1, 0], smooth_path[i+1, 1])
    smooth_path_with_heading[i, 3] = curvature


# ======================================================================================
# 显示结果
print("Smooth Path with Heading Matrix Dimensions: ", smooth_path_with_heading.shape)
# print(smooth_path_with_heading)
# 打印前五组数据
print("First five data points of the smooth path:")
# print(smooth_path_with_heading[:5])

# 打印最后五组数据
print("Last five data points of the smooth path:")
# print(smooth_path_with_heading[-5:])



# ================================================================================
import matplotlib.pyplot as plt
plt.figure()
plt.imshow(mapData_bak, origin='lower', cmap='gray')
# plt.plot(routeY, routeX, 'y.')  # 原始路径
plt.plot(routeYSmooth, routeXSmooth, 'r-', linewidth=2)  # 平滑后的路径
plt.plot(start[1], start[0], 'go')  # 起点
plt.plot(goal[1], goal[0], 'bo')  # 终点
plt.show()

smooth_path_with_heading = smooth_path_with_heading[0:smooth_path_with_heading.shape[0]+1, 0:smooth_path_with_heading.shape[1]]
smooth_path_with_heading = smooth_path_with_heading.transpose()
# smooth_path_with_heading[[0, 1], :] = smooth_path_with_heading[[1, 0], :]
savemat('traj_diySYSU.mat', {'trajSYSU': smooth_path_with_heading})