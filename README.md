
传感器外参标定本质上是获得两个传感器的位移量(x, y, z)和旋转量（roll,pitch,yaw），三维空间中可以用一个齐次变换矩阵（`Homogeneous transformation matrix`）来描述这样的变换关系，在三维数据处理领域，点云配准（Point Cloud Registration）就是用于处理两个点云间位姿匹配问题的一类方法，其中NDT(`Normal Distribution Transform`，正态分布变换)和ICP(`Iterative Closest Point`, 迭代最近点算法)是其中的代表。本文主要基于NDT算法介绍多激光雷达外参标定的方法和实践。

> 齐次变换矩阵：描述两个坐标系间的平移和旋转变换关系的$4\times4 $ 矩阵，给定两个坐标系的平移量 $(x_t, y_t, z_t)$和欧拉角 $R(\alpha, \beta, \gamma)$，一个3D的齐次变换矩阵写作：
> ![在这里插入图片描述](https://img-blog.csdnimg.cn/20200505121244173.png)
> 其中，$\alpha$ 为 yaw*，$\beta$*为 pitch，$\gamma$ 为 roll。

#### 正态分布变换

正态分布变换是一种用正态分布函数来描述一个体素网格（voxel）内的点的方法，令$P=\left\{p_{i} \mid i=0,1, \ldots, t-1\right\}$  = $\left\{\left(x_{i}, y_{i}, z_{i}\right) \mid i=0,1, \ldots, t-1\right\}$ 表示一个包含 *t* 个点的点云个点的点云，NDT算法首先使用三维的网格（也称为体素）将点云进行划分，如下图所示，我们称这类体素为ND体素：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200505121312396.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu,size_16,color_FFFFFF,t_70#pic_center)

假定ND体素 *k* 中包含有*m* 个点，那么这个ND体素中所有点的均值 $\mu_k$ 和协方差矩阵 $\Sigma_k$计算公式为:

$$
\mu_{k}=\frac{1}{m} \sum_{i=0}^{m-1} p_{k i}
$$

$$
                \Sigma_{k}=\frac{1}{m} \sum_{i=0}^{m-1}\left(p_{k i}-\mu_{k}\right)\left(p_{k i}-\mu_{k}\right)
$$

其中$$p_{k i}$$为ND体素 `k`中的点 *i*，即 $p_{k i}=\left(x_{k i}, y_{k i}, z_{k i}\right)$，那么该ND体素内点的概率密度函数 *f(k)* 可以表示为：

​                                                                      $f(k)=\frac{1}{(2 \pi)^{\frac{3}{2}} \sqrt{\left|\Sigma_{k}\right|}} e^{-\frac{\left(p_{k}-\mu_{k}\right)^{T} \Sigma_{k}^{-1}\left(p_{k}-\mu_{k}\right)}{2}}$

按照此方法，可以将整个目标点云使用ND体素进行划分，并且计算每一个ND体素内的正态分布参数。NDT匹配的准确度和ND体素格的划分尺寸相关**，采用的ND体素尺寸越小，相应的NDT匹配精度也会越高，**所以在利用NDT算法进行扫描匹配定位时需要对匹配精度和算法实时性间进行取舍，我们使用NDT进行多激光雷达标定为离线任务，所以可以将ND Voxel设置相对小一些以提高最终的精度。

> 在利用NDT继续激光雷达扫描匹配定位中，考虑到城市道路场景下地图的尺寸通常较大，采用高密度的地图尺寸进行划分将造成内存占用偏高的问题，并且使得匹配的计算量增大，无法满足定位的实时性要求。所以在自动驾驶定位的场景中，通常使用较大尺寸的ND体素格配合高密度的点云地图来平衡NDT匹配定位的实时性和准确性。

#### NDT匹配参数

> NDT匹配定位算法定义了一些匹配参数，通过这些匹配参数可以将输入点云进行三维坐标变换，得到输入点云到目标点云中的变换关系，激光雷达的外参标定，目标点云实际上就是目标激光雷达（ROS TF中的parent frame），输入点云就是要标定到目标激光雷达坐标系下的激光雷达的点云（ROS TF中的child frame）。

NDT算法中一共有两组配置参数，分别是旋转参数向量$(\alpha, \theta, \gamma)^{T}$ 和平移参数向量 $(d^{x},d^y,d_z)$ ,旋转参数向量表示变换后的点云相对当前输入点云在姿态角上的旋转量，平移参数向量表示在*(x, y, z)*三个方向的平移量。那么对于输入点云而言，每个点进行三维坐标转换的计算公式如下：

$p_{i}^{\prime}=R p_{i}+d$

其中 *R* 为旋转矩阵，其计算公式为：

$R=\left(\begin{array}{ccc}\cos \alpha & -\sin \alpha & 0 \\ \sin \alpha & \cos \alpha & 0 \\ 0 & 0 & 1\end{array}\right) \times\left(\begin{array}{ccc}\cos \beta & 0 & \sin \beta \\ 0 & 1 & 0 \\ -\sin \beta & 0 & \cos \beta\end{array}\right) \times\left(\begin{array}{ccc}\cos \gamma & -\sin \gamma & 0 \\ \sin \gamma & \cos \gamma & 0 \\ 0 & 0 & 1\end{array}\right)$

所以对于NDT匹配算法而言，配准的目标就是找到一组三维坐标系变换参数向量，使得输入点云在经过这组参数指定的旋转和平移之后，能够和目标点云拟合。我们称参数向量 $\theta$ 为NDT匹配参数，接下来，将介绍如何搜索找到这一组参数。

#### NDT扫描匹配算法

NDT算法首先对目标点云进行正态分布变换，得到目标点云的所有ND体素，接着需要输入当前输入点云的初始位姿*（x, y, z, roll, pitch, yaw）*，将此位姿作为初始搜索位置。在传感器标定中，这个初始变换位姿也就是我们对于多个激光雷达平移量和旋转量的粗略估计，可以通过简单的测量得到（精度不需要太高，我们最终将通过NDT算法得到高精度的结果）。

初始位姿是对初始点云在目标点云中的位姿的估计，这个估计值可以帮助NDT算法中的参数优化方法迅速收敛。得到初始估计后，开始搜索一组最优的NDT匹配参数，使得输入点云在通过这组参数变换后和目标点云拟合度最高。使用如下公式来描述两个点云的拟合度：

$$
Fitness(P, \theta)=\sum_{i=0}^{n-1} e^{\frac{-\left(p_{i}^{\prime}-\mu_{i}\right)^{T} \Sigma_{i}^{-1}\left(p_{i}^{\prime}-\mu_{i}\right)}{2}}
$$
其中，$p{\prime}$是输入点云在经过三维坐标变换参数 θ 转换后的点，$\mu_i$ 是与该输入点相对应的目标点云ND体素格的均值，$\Sigma_i$ 是相应的ND体素内的协方差矩阵。拟合度$Fitness(P,\theta)$ 数值越大说明输入点云和目标点云在该位置越匹配，**通过搜索参数向量 θ 得到大的拟合度**，这是一个典型的非线性最优化问题，NDT算法使用`牛顿法`来求解最优参数。牛顿法是一种最小化目标函数的方法，所以目标函数$f(\theta)$为：$Fitness(P,\theta)$

> 其中$\theta=\left(\alpha, \theta, \gamma, d_{x}, d_{y}, d_{z}\right)^{T}$， 是输入点云到目标点云的三维坐标转换参数。通过迭代牛顿法，不断调整 θ向量，使得$Fitness(P,\theta)$小于一个阈值，则称NDT参数优化已经收敛，根据此时的变换参数向量 θ\ 即可确定输入点云在目标点云中的姿态，**也就是lidar A到lidar B的TF关系。**
