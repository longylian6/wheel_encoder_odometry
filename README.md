# Kinematics
两轮差速轮机器人在全局参考坐标系下的位姿为 $[x, y, \theta]^T$ ，其中 $x$ 和 $y$ 表示机器人的位置， $\theta$ 表示机器人的航向角，其运动学模型如下所示：

$$
\begin{aligned}
    \dot{x} &= v \cos \theta \\
    \dot{y} &= v \sin \theta \\
    \dot{\theta} &= \omega 
\end{aligned} \tag{1}
$$

其中 $v$ 为线速度， $\omega$ 为角速度，机器人通过线速度和角速度来控制。

![encoder_odom](images/encoder_odom.png)

安装在机器人左右两边轮子上的编码器可以测量出轮子在 $\Delta t$ 时间内的转动角度，已知轮子的半径，可以计算在 $\Delta t$ 时间内机器人的左右轮子走过的路程 $\Delta s_l$ 和 $\Delta s_r$ ，假设两个轮子之间的距离为 $L$ ，则机器人在 $\Delta t$ 时间内的航向角变化量为：

$$
\Delta \theta = \frac{\Delta s_l}{R - \frac{L}{2}} = \frac{\Delta s_r}{R + \frac{L}{2}} = \frac{\Delta s}{R} \tag{2}
$$

其中， $R$ 为转弯半径。根据公式 $(2)$ ，可以计算出机器人在 $\Delta t$ 时间内的航向角变化量 $\Delta \theta$ 为

$$
\Delta \theta = \frac{\Delta s_r - \Delta s_l }{L} \tag{3}
$$

机器人在 $\Delta t$ 时间内的路程变化量 $\Delta s$ 为

$$
\Delta s = \frac{\Delta s_r + \Delta s_l }{2} \tag{4}
$$

以下介绍两种方法，根据编码器得到的左右轮子走过的路程 $\Delta s_l$ 和 $\Delta s_r$ ，计算机器人在参考坐标系下的位姿 $[x, y, \theta]^T$ 。
### 方法1

公式 $(1)$ 离散化后的模型如下：

$$ 
\begin{aligned}
    x_{k+1} &= x_k + \Delta t \cdot v_k \cos \theta_k \\
    y_{k+1} &= y_k + \Delta t \cdot v_k \sin \theta_k \\
    \theta_{k+1} &= \theta_k + \Delta t \cdot \omega_k 
\end{aligned} \tag{5}
$$

根据假设，机器人在 $k$ 时刻的航向角变化量 ${\Delta \theta}_k = \Delta t \cdot \omega_k$ ，机器人在 $k$ 时刻的路程变化量 ${\Delta s}_k = \Delta t \cdot v_k$ ，结合公式 $(3)$ 和公式 $(4)$ ，可知：

$$ 
\begin{aligned}
    x_{k+1} &= x_k + \frac{\Delta s_r + \Delta s_l }{2} \cos \theta_k \\
    y_{k+1} &= y_k + \frac{\Delta s_r + \Delta s_l }{2} \sin \theta_k \\
    \theta_{k+1} &= \theta_k + \frac{\Delta s_r - \Delta s_l }{L} 
\end{aligned} \tag{6}
$$

