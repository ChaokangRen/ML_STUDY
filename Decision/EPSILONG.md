# 基于POMDP的决策规划方案调研
## 论文2：EPSILON: An Efficient Planning System for Automated Vehicles in Highly Interactive Environments
### PipeLine
该文章的主要架构如下图所示：  

![pipeline](/Decision/elements/EPSILON_pipeline.png "pipeline")    
  
### Problem Formulation
做如下定义：  
$\xi_t$:表示以自车为中心的局部环境,包括道路结构、交通灯以及静态障碍物  
$x^i_t$表示第i辆车在时间t时的状态，不失一般性，i=0表示自车。  
对于时间t的规划周期内，自车接收到的观测为 $z_t$,并且使用该观测来估计真实的状态$x_t$  
$\left<z_t,\xi_t\right>$:表示整个行为规划层的输入  
$ \mathcal{D}_t$:表示行为层输出，由序列化的状态$[x_{t+1},x_{t+2},...,x_{t+H}]$,表示在整个规划时域H下，所有车辆的状态。  
$\left<\mathcal{D}_t,\xi_t\right>$:表示整个运动规划层的输入  
运动规划层的输出为一条光滑的参数化的轨迹。

---

一个POMDP问题可以由以下tuple定义： $\left<\mathcal{X,A,Z,T,O,R}\right>$,其中，$\mathcal{X},\mathcal{A},和\mathcal{Z}$分别表示状态空间，动作空间和观测空间。  
$T(x_{t-1},a_t,x_t) = p(x_t|x_{t-1},a_t)$:表示状态转移概率函数模型
$O(x_t,z_t) = p(z_t|x_t):观测函数模型  

$\mathcal{R}:\mathcal{X} \times \mathcal{A} \rightarrow \mathbb{R}$：表示奖励函数，agent在状态$x_{t-1}$下采取动作$a_t$的奖励。  
由于真实世界中，一些状态无法直接获得，比如隐藏的意图、随机的噪声等等，所以POMDP需要一个信念状态(belief)$b\in \mathcal{B}$,表示$\mathcal{X}$中的状态的概率分布。belief可在agent离开初始belief$b_{t-1}$后被更新，通过采取动作$a_t$，收到$z_t$的观察。belief的状态可以由以下贝叶斯公式推导：

$$
\begin{equation}
b_t(x_t) = p(x_t|z_t,a_t,b_{t-1})= \eta O(x_t,z_t) \int_{x_{t-1}\in \mathcal{X}}T(x_{t-1,a_t,x_t})b_{t-1}(x_{t-1})dx_{t-1}
\end{equation} 
$$
$\eta$表示 a normalizing factor