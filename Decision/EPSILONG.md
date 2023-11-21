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
$\eta$表示 a normalizing factor. POMDP就是找个一个最优策略 $\pi^*$ ,将信念状态映射到一个动作 $\pi:  \mathcal{B} \rightarrow \mathcal{A}$,在规划时域最大化总的折扣奖励：

$$
\begin{equation}
\pi^*:=\argmax_{\pi} \mathbb E \left[
    \sum^{t_H}_{t=t_0} \gamma^{t-t_0}R(x_t,\pi(b_t))|b_{t_0}
\right]
\end{equation}
$$
$t_0$ 是当前的规划时间，$\gamma$表示折扣因子。对于在线POMDP来说，开始于初始的信念空间 $b_{t_0}$,在确定的规划时域中，并随着动作空间 $\mathcal{A}$ 和观测空间 $\mathcal{Z}$ 逐步的扩大，一个节点一个节点的构建出信念树。那么，通过在每个内部的节点应用贝尔曼方程，即可找出一条最优策略。
$$
\begin{equation}
V^*(b) = \max_{a \in \mathcal{A}} Q^*(b,a) = \max_{a \in \mathcal{A}} 
\left\{
 \int_{x \in \mathcal{X}} b(x)R(x,a){\rm}dx  +
 \gamma \int_{z\in \mathcal{Z}}p(z|b,a)V^*(\tau(b,a,z)){\rm d}z
\right\}
\end{equation}
$$

其中，$V^*(b) = \int_{s \in S}V^*(s)b(s)ds$ is the optimal utility functin for the belief state
不同于之前的工作仅使用单一的最优动作在初始节点上作为最终输出，这里我们在信念树上得到一条完整的轨迹(trace) $\mathcal{S}_t = [b^*_t,a^*_{t+1},z^*_{t+1},b^*_{t+1},...,b^*_{t+H}] $，通过在动作分支和观测分支上循环构建以下优化目标
$$
\begin{equation}
\begin{aligned}
a^*_t = \argmax_{a_t \in \mathcal{A}}Q^*(b_{t-1},a_t) \\
z^*_t = \argmax_{z_t \in \mathcal{Z}}p(z_t|b_{t-1},a^*_t)
\end{aligned}
\end{equation}
$$
生成的轨迹包含最优动作 $a^*_t$ 在每个信念节点 $b^*_{t-1}$,同时在动作 $a^*_t$ 下得到最有可能的观测 $z^*_t$. 最终的决策 $\mathcal{D}_t$ 可以通过在 $S_t$ 每一步 应用 $x_t = \argmax_x b^*_t(x)$得到。相比于一个单一的优化动作， $\mathcal{D}_t$ 包含更多关于环境和未来预报中完整的信息，这在接下来的运动规划中是必不可少的。
考虑在t时刻有N个agents的驾驶场景，所有的状态可以写为，$x_t={x^0_t,x^1_t,...,x^N_t} \in \mathcal{X}$,其中， $x^i_t \in \mathcal{X}^i$表示第i辆车的状态，包括位置，车速，加速度，朝向以方向盘转角，另外对于周围的车辆，一些隐藏的信息如驾驶意图、驾驶激进程度，虽然无法被直接观测到，但也需要包含到状态中。在这个驾驶场景中，我们仅能控制自车的油门刹车和转向，也就是说无法直接的决定其他agnet的动作。因此原始问题中的动作可以重新表示为 $a_t = a^0_t$ ,同样的动作空间 $\mathcal{A} = \mathcal{A}^0$. 完整的状态转移模型T可以表示为完整状态的联合分布: $p(x_t|x_{t-1},a_t) = p(x^0_t,...,x^N_t|x^0_{t-1},...,x^N_{t-1},a^0_t)$这对于直接建模来说并不简单.  
然后，由于在真实交通环境中，大多数其他的agents，比如vehicle和bicycle，都遵循了交通规则和物理定律，可以简化上面的公式通过对周围agent做出合理的假设，将其转为多智能体交互的模型。我们对周围每个agent赋予一个条件概率转移模型，伴随着相应的动作 $a^i_t \in \mathcal{A}^i,\forall i \neq 0$,并且假设每个agent的瞬时转移是独立的，于是有：

$$
\begin{equation}
\large{p(x_t|x_{t-1},a_t)} \approx 
\begin{matrix} 
\underbrace 
{\large{p(x^0_t|x^0_{t-1},a^0_t)}} \\{\scriptsize{ego\,transition}}
\end{matrix}

\prod^N_{i=1} \int_{\mathcal{A}^i}

\begin{matrix} 
\underbrace 
{\large{p(x^i_t|x^i_{t-1},a^i_t)}}
\\\scriptsize{i-th\,agent's\,transition}
\end{matrix}
\begin{matrix} 
\underbrace 
{\large{p(a^i_t|x_{t-1}) }} \\ \scriptsize{driver model}
\end{matrix}
{\rm d}a^i_t
\end{equation}
$$

该公式将被控车辆和其他agents区别开来，在公式中，$p(x^i_t|x^i_{t-1},a^i_t)$是其他代理的假设转换模型，反映代理的低级运动学， $p(a^i_t|x_{t-1})$是假设的驾驶员模型，代表其他智能体的高级决策过程，根据驾驶环境提供适当的控制信号。驾驶员模型可以是用户定义的，也可以是通过数据驱动来学习的，可以通过一些潜在的状态如意图或者激进度来控制，实现不同的操作或驾驶风格。
注意到，自车的动作 $a^0_t$时通过预定义的自车策略生成的。自车的观测 $z^0_t = z_t$包括感知模块估计的其他车辆的位置和速度。对于周围的agents,对于周围的智能体，我们使用它们的位姿作为原点，并将自我观察 $z^0_t$ 转换到它们的坐标系中作为观察 $z^i_t$。由于 $z^i_t$ 完全取决于自车的观测，假设观测过程是独立的，我们可以得到： $p(z_t|x_t) = \prod^N_{i=1}p(z^i_t|x^i_t)$.于是，式(1)可以写为：

$$
\begin{equation}
b_t(x_t) = \eta \cdot 
\begin{matrix} \scriptsize{belief\,update\,for\,ego\,agent} \\
\overbrace{
p(z^0_t|x^i_t)\int_{\mathcal{X}^0}p(x^0_t|x^0_{t-1},a^0_t)b^0_{t-1}(x^0_{t-1}){\rm d}x^0_{t-1}
}
\end{matrix}\cdot
\,\,
\prod^N_{i=1}
\begin{matrix} \scriptsize{belief\,update\,for\,other\,agent} \\
\overbrace{
p(z^i_t|x^i_t) \iint_{\mathcal{X}^i \mathcal{A}^i}
p(x^i_t|x^i_{t-1},a^i_t)p(a^i_t|x_{t-1})b^i_{t-1} {\rm d}a^i_t {\rm d}x^i_{t-1}
}
\end{matrix}
\end{equation} 
$$
尽管每个agent的状态转移是独立的，假设的驾驶员模型 $p(a^i_t|x_{t-1})$ 和观测模型 $p(z^i_t|x^i_t)$ 利用了所有agnets的状态和观测，所以信念的更新是一个交互的过程。请注意，隐藏状态是在信念更新期间使用观测值逐步估计的，这正是行为预测的作用。因此，POMDP 隐式包含预测，表明规划和预测是自然耦合的。还值得注意的是，将预测和规划分离的方法本质上是原始 POMDP 公式的简化。通过循环应用置信更新，我们可以从初始置信节点开始构建置信树并提取最终决策 $\mathcal{D}_t$.然而，置信树的规模随树深度呈指数增长，这对于实时应用程序来说在计算上是难以处理的。为了克服这个问题，在本文中，我们将领域知识应用到公式中，以进一步简化在快速变化的驾驶环境中实现实时决策的问题，同时保留处理交互和不确定性的能力。给定行为规划器 $\mathcal{D}_t$的输出，运动规划层旨在生成安全且平滑的轨迹，以细粒度的方式实现高层决策。请注意，由于我们的行为层是在多智能体设置中制定的，因此它自然会推理所有智能体的未来状态。因此，运动规划的作用可以归结为局部轨迹优化问题，如图2所示。  
![MotionPlanning](/Decision/elements/Epsilon_mp_ssc.png "SSC")   