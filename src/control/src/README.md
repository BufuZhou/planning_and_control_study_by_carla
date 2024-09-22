# 物理模型
## 横向模型
参考书籍Vehicle Dynamics and Control by Rajesh Rajamani中的第二章。

![lateral vehicle dynamics](/docs/control/lateral_vehicle_dynamics.png)

基于动力学的路面误差模型具体如下：
$$
\frac{d}{dt} 
\begin{bmatrix} 
e_1 \\ 
\dot{e_1} \\ 
e_2 \\ 
\dot{e_2} \\
\end{bmatrix}
 = \
\begin{bmatrix} 
0 & 1 & 0 & 0 \\ 
0 & -\frac{2C_{af}+2C_{ar}}{mV_x} & \frac{2C_{af}+2C_{ar}}{m} & -\frac{2C_{af}l_f+2C_{ar}l_r}{mV_x} \\
0 & 0 & 0 & 1 \\ 
0 & -\frac{2C_{af}l_f - 2C_{ar}l_r}{I_zV_x} & \frac{2C_{af}l_f - 2C_{ar}l_r}{I_z} & -\frac{2C_{af}{l_f}^2 + 2C_{ar}{l_r}^2}{I_zV_x} \\
\end{bmatrix}
\begin{bmatrix} 
e_1 \\ 
\dot{e_1} \\ 
e_2 \\ 
\dot{e_2} \\
\end{bmatrix}
+
\begin{bmatrix} 
0 \\ 
\frac{2C_{af}}{m} \\ 
0 \\ 
\frac{2C_{af}l_f}{I_z} \\
\end{bmatrix}
\delta
+
\begin{bmatrix} 
0 \\ 
-\frac{2C_{af}l_f - 2C_{ar}l_r}{mV_x} - V_x\\ 
0 \\ 
-\frac{2C_{af}l_f^2 + 2C_{ar}l_r^2}{I_zV_x} \\
\end{bmatrix}
\dot{\psi}_{des}
$$
## 纵向模型
 纵向⽅向使⽤的模型很简单，可以简单的理解为⼀个纵向的PID, 状态⽅程如下所示。
 $$
 \frac{d}{dt} 
 \begin{bmatrix} 
 e_s \\ 
 \dot{e_s} \\ 
 \end{bmatrix}
 = \
 \begin{bmatrix} 
 0 & 1 \\ 
 0 & 0 \\
 \end{bmatrix}
 \begin{bmatrix} 
 e_s \\ 
 \dot{e_s} 
 \end{bmatrix}
  + 
 \begin{bmatrix} 
 0 \\ 
 -1 \\
 \end{bmatrix}
 u_a
 $$
 纵向距离的导数等于纵向速度, 并且该等式主要是为了能够帮助整个MPC状态⽅程去凑对应的标 准矩阵范式而存在的。
 $$
 \frac{d{e_s}}{dt} = \dot{e_s} = e_v
 $$
 重点解释第二个等式，如下所示。这里更多的是一种PID的思想。
 $$
 \dot{e_s} = -u_a
 $$
 将上式离散化，如下所示。当速度误差在正向的变⼤，实际纵向速度大于目标纵向速度，也就是我们需要施加⼀个负向的加速度，使得速度误差降下来；反之，则需要施加⼀个正向的加速度，使得我们的速度增加，从而降低我们的速度误差。
 $$
 \frac{e_v(k + 1)- e_v(k)}{\Delta t} = -u_a
 $$

 ## 模型预测的物理模型
结合前面的横向模型和纵向模型，模型预测控制所需的物理模型具体如下：
$$
\frac{d}{dt} 
\begin{bmatrix} 
e_1 \\ 
\dot{e_1} \\ 
e_2 \\ 
\dot{e_2} \\
e_s \\
\dot{e_s} \\
\end{bmatrix}
 = \
\begin{bmatrix} 
0 & 1 & 0 & 0 & 0 & 0 \\ 
0 & -\frac{2C_{af}+2C_{ar}}{mV_x} & \frac{2C_{af}+2C_{ar}}{m} & -\frac{2C_{af}l_f+2C_{ar}l_r}{mV_x} & 0 & 0\\
0 & 0 & 0 & 1 & 0 & 0 \\ 
0 & -\frac{2C_{af}l_f - 2C_{ar}l_r}{I_zV_x} & \frac{2C_{af}l_f - 2C_{ar}l_r}{I_z} & -\frac{2C_{af}{l_f}^2 + 2C_{ar}{l_r}^2}{I_zV_x} & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
\end{bmatrix}
\begin{bmatrix} 
e_1 \\ 
\dot{e_1} \\ 
e_2 \\ 
\dot{e_2} \\
e_s \\
\dot{e_s} \\
\end{bmatrix}
+
\begin{bmatrix} 
0 & 0\\ 
\frac{2C_{af}}{m} & 0 \\ 
0 & 0\\ 
\frac{2C_{af}l_f}{I_z}  & 0\\
0 & 0 \\
0 & -1 \\
\end{bmatrix}
\begin{bmatrix}
\delta \\
a \\ 
\end{bmatrix}
+
\begin{bmatrix} 
0 \\ 
-\frac{2C_{af}l_f - 2C_{ar}l_r}{mV_x} - V_x\\ 
0 \\ 
-\frac{2C_{af}l_f^2 + 2C_{ar}l_r^2}{I_zV_x} \\
0 \\
0 \\
\end{bmatrix}
\dot{\psi}_{des}
$$
上述方程转化为矩阵形式：
$$
\dot{x} = Ax + Bu + C
$$
其中，$x$是状态向量($m{\times}1$)，$u$是控制向量($n{\times}1$)，$A$是系统矩阵($m{\times}m$)，$B$是控制矩阵($m{\times}n$)，$C$是常量向量($m{\times}1$)，其中$m=6,n=2$，各项参数具体如下：
$$
x = \begin{bmatrix} 
e_1 \\ 
\dot{e_1} \\ 
e_2 \\ 
\dot{e_2} \\
e_s \\
\dot{e_s} \\
\end{bmatrix},\quad
u = \begin{bmatrix}
\delta \\
a \\ 
\end{bmatrix},\quad
A = 
\begin{bmatrix} 
0 & 1 & 0 & 0 & 0 & 0 \\ 
0 & -\frac{2C_{af}+2C_{ar}}{mV_x} & \frac{2C_{af}+2C_{ar}}{m} & -\frac{2C_{af}l_f+2C_{ar}l_r}{mV_x} & 0 & 0\\
0 & 0 & 0 & 1 & 0 & 0 \\ 
0 & -\frac{2C_{af}l_f - 2C_{ar}l_r}{I_zV_x} & \frac{2C_{af}l_f - 2C_{ar}l_r}{I_z} & -\frac{2C_{af}{l_f}^2 + 2C_{ar}{l_r}^2}{I_zV_x} & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
\end{bmatrix},\quad
B = \begin{bmatrix} 
0 & 0\\ 
\frac{2C_{af}}{m} & 0 \\ 
0 & 0\\ 
\frac{2C_{af}l_f}{I_z}  & 0\\
0 & 0 \\
0 & -1 \\
\end{bmatrix},\quad
C = \begin{bmatrix} 
0 \\ 
-\frac{2C_{af}l_f - 2C_{ar}l_r}{mV_x} - V_x\\ 
0 \\ 
-\frac{2C_{af}l_f^2 + 2C_{ar}l_r^2}{I_zV_x} \\
0 \\
0 \\
\end{bmatrix}
\dot{\psi}_{des}
$$
在百度apollo的mpc控制器中，上述公式对应的变量具体如下：
$$
matrix\_a\_ = A,\quad matrix\_b\_ = B,\quad matrix\_c\_ = C
$$

# 连续方程离散化
实际工程计算中都是数值计算，因此需要对连续方程进行离散化。当前的连续方程具体如下：
$$ \dot{x} = Ax + Bu + C $$
$x$是和时间相关的函数，已知当前时刻的$x(t)$值，计算下一个时刻的值$x(t+\Delta t)$。求解x(t+Δt)本质就是根据初始值和微分项，计算${\Delta}t$内的积分值。常见计算积分的方法有三种，向前欧拉法，向后欧拉法，中点欧拉法（梯形积分），三者的不同在于微分值的选择。积分公式具体如下：

向前欧拉法： $x(t+{\Delta}t) = x(t) + \dot{x}(t){\Delta}t$

向后欧拉法： $x(t+{\Delta}t) = x(t) + \dot{x}(t+{\Delta}t){\Delta}t$

中点欧拉法： $ x(t+{\Delta}t) = x(t) + \frac{1}{2}(\dot{x}(t) + \dot{x}(t+{\Delta}t)){\Delta}t$

上述连续方程对应离散化方程，$x(t)$记作$x_n$,下个
时刻的值$x(t+\Delta t)$记作$x_{n+1}$, $\Delta t$记作采样周期$T$。
对于前向欧拉法，则有：
$$
x_{k+1} = x_k + (Ax_k + Bu_k + C)T \\
x_{k+1} = (AT + I)x_k + (BTu_k + CT)
$$
对于后向欧拉法，则有：
$$
x_{k+1} = x_k + (Ax_{k+1} + Bu_{k+1} + C)T \\
(I - AT)x_{k+1} = x_k + (Bu_{k+1}T + CT) \\
x_{k+1} = (I - AT)^{-1}x_k + (I - AT)^{-1}(Bu_{k+1}T + CT)
$$
对于中点欧拉法，则有：
$$
x_{k+1} = x_k + \frac{1}{2}(Ax_k + Bu_k + C + Ax_{k+1} + Bu_{k+1} + C)T \\
(I - \frac{A}{2})x_{k+1} = (I + \frac{A}{2})x_k + \frac{BT}{2}(u_k + u_{k+1}) + CT \\
x_{k+1} = (I - \frac{A}{2})^{-1}(I + \frac{A}{2})x_k + (I - \frac{A}{2})^{-1}\frac{BT}{2}(u_k + u_{k+1}) + (I - \frac{A}{2})^{-1}CT
$$
在百度apollo的mpc控制器中，连续方程的离散化比较奇怪，$x_k$的系数采用的是中点欧拉法，$B$和$C$项的系数是前向欧拉法,具体如下：
$$ x_{k+1} = (I - \frac{A}{2})^{-1}(I + \frac{A}{2})x_k + BTu_k + CT
$$
上述方程转化为矩阵形式：
$$ x_{k+1} = A_kx_k + B_ku_k + C_k \\ $$
其中，$x_k$ 和$x_{k+1}$是状态向量($m{\times}1$)，$u_k$是控制向量($n{\times}1$)，$A_k$是系统矩阵($m{\times}m$)，$B_k$是控制矩阵($m{\times}n$)，$C_k$是常量向量($m{\times}1$)，其中$m=6,n=2$，各项参数具体如下：
$$
A_k = (I - \frac{A}{2})^{-1}(I + \frac{A}{2}), \quad B_k = BT, \quad C_k = CT
$$
上述公式在代码中对应的变量具体如下：
$$
matrix\_state\_ = x_k, matrix\_ad\_ = A_k,\quad matrix\_bd\_ = B_k,\quad 
matrix\_cd\_ = C_k
$$












