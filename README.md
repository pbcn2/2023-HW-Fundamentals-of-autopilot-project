# 2023-HW-Fundamentals-of-autopilot-project
 2023 SYSU-Fundamentals-of-autopilot-project homework

## 教程

本项目处于debug状态( 2023.7.2 )

目前是由两个部分整合而成

- 前端-规划
- 后端-控制

### <font color=red>前端</font>

前端算法库位于`Front End - Planning`文件夹，多种函数实现了主体的路径规划功能

#### Astar算法

A\*文件夹中的 `v1.m` `v2.m` `v3.m`三个函数实现了手动的A*路径查找算法，具体如下：

`v1.m`文件实现了A\*算法，对路径进行规划，同时引入障碍物膨胀机制，对障碍物进行膨胀，使得生成的路径远离障碍物，降低小车碰撞的可能性

`v2.m`在此基础上对A\*进行了步长放大，使得搜索速度显著加快，并且能够为后来引入三次、五次插值曲线做好准备

`v3.m`由对搜索步长进行了调整，使得在转角处的步长更小，意味着能够使得路径点更加密集，以防止出现相邻两点连线穿过障碍物的情况

#### RRTstar算法

`RRT_star.m`实现了对图片调用RRT_star算法进行路径规划，在实验过程中能够有效找到路径。但是路径转折点过多，且迭代生成的时间过长，尤其是寻路过程执行到后期的时候有效点生成效率越来越低，时间开销难以接受。因此最终放弃此种算法。

#### 辅助函数

`compute_psi.m`函数实现了对当前路径的航向角的计算，具体来讲，以水平向右的角度为零点，向竖直上方偏转记作正数，向下偏转记作负数

`compute_curvature.m`函数实现了对航线曲率的计算，因为是离散点，所以无法直接通过求导等方法实现，在此使用了相邻三个点求曲率的算法，具体见代码。特别的是，面向航行方向，向左弯曲为正，向右弯曲为负。

#### 具体实现：Python 版本

`py_v2.py`是具体使用的函数，在这个函数中，基础的路径搜索算法使用的是Astar，使用障碍物膨胀的方法解决离障碍物太近的问题，在算出路径以后，使用滑动窗口平均的方法对路径进行平滑化，效果较好。且本人使用python较matlab熟练很多，最终采用了这一版的路径规划



### <font color=red>后端</font>

本模块使用simulink设计了一个lqr控制器，控制小车追踪路径。





## REFERENCE



[自动驾驶控制算法：Carsim和Simulink联合仿真实现LQR最优控制轨迹跟踪策略](https://www.bilibili.com/video/BV1JR4y1C7rJ/?spm_id_from=333.1007.top_right_bar_window_history.content.click&vd_source=0b9cca431f65b84627b0251721c328db)<font color=red>( Greatest contribution ) </font>

[横向控制器LQR算法详解](https://zhuanlan.zhihu.com/p/616341034)

[横向控制算法与流程图（基于动力学模型的LQR）](https://blog.csdn.net/ChenGuiGan/article/details/122993738)

[【Carsim Simulink自动驾驶仿真】基于MPC的动力学控制](https://blog.csdn.net/w_w_y/article/details/124047232)

[详细介绍用MATLAB实现基于A*算法的路径规划](https://blog.csdn.net/qq_44339029/article/details/108682933)

[全局路径规划A star的Matlab实现](https://blog.csdn.net/gophae/article/details/103060966)

