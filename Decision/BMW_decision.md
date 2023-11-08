# 论文3：Automated Driving in Uncertain Enviroments:Planning with Interaction and Uncertain Maneuver Prediction
如图 1 所示，我们定义了其他道路使用者未来轨迹不确定的四个关键原因。
* 其他驾驶员的意图未知
* 它们未来的纵向运动未知
* 与自车交互的可能性
* 传感器的噪音
![fig1](/Decision/elements/BMW_fig1.png "fig1")  
这篇文章的主要工作就是为自动驾驶展示了一个在线的POMDP算法。
## Related Work
### D. Belief state planning
belief state规划表示一个规划问题，仅能得到当前状态的概率分布，而非状态本身。允许对不确定行为和不确定的动作建模，通过概率转移模型(probabilistic transition model).与本节中介绍的相关工作相比，我们的目标是提出一个结合多个方面的 POMDP 统一规划框架:
* 它通常适用于任何交叉口几何形状和可变数量的交通参与者
* 它考虑当前甚至预测其他交通参与者意图/运动的未来不确定性，并且不依赖（但可以从中获利）车辆对车辆（V2V）通信
* 它是在线且随时可用的。 给予更多的时间或计算能力，结果将会改善
* 运行在一个连续的状态空间中。

### Problem Statement
这项工作的重点是自我车辆的在线决策，即生成一系列期望的加速度 $a_0 = (a^{t_0}_0,a^{t_1}_0,a^{t_2}_0,...),用于穿越一个任意布局和未知其他交通参者的数量以及意图的无信号灯的路口。
假设自身车辆r0的路径关于静态障碍物是无碰撞的，并且由路径规划器先验地生成或者简单地从给定地图的道路几何形状中检索。第二部，纵向速度是通过我们的算法沿着r0计算出的。这种做法在文献[33]中被称为路径速度分解，并将轨迹规划问题简化为一维工作空间。   
环境有一系列的agents组成，$ \mathcal{N} = {N_0,...,N_K}，自车下标为0.对于其他agent，都有一组未来路径的假设。自车与其他车的路径假设有一个拓扑地图 $\mathcal{R}= {r^{(0)},r^{(1)},...,r^{(I)}}$,其中， $r^{(i)} = \left\{\overrightarrow{q_{i,0}q_{i,1}},...,\overrightarrow{q_{i,J-1}q_{i,J}}\right\}$,$q_{i,j}$表示路径i的第j个点的位置坐标。每个agent都有一个相应的路径映射，$r_k : N_k$ ,该agent的速度为 $v_k(t) \in [0,v_{max}]$.每条路径都有一组后续路径假设 $\mathcal{M}^{(i)} = succ(r^{(i)})$.agent从当前路径通过未知的概率 $P(r'_k|r_k)$ 转移到下一个路径 $r'_k$.   
<font color=red>
note:一辆车可能有多个潜在的路线，但最终执行的时候只会有一条。  
</font>
由于各个路线元素可能相互交叉，因此交叉函数 $c(r_i,r_j)$ 定义为:
$$
\begin{equation}
c(r^{(i)},r^{(j)}) =
\left\{
\begin{aligned}
1,if\,\,r^{(i)}\cap r^{(j)} \neq \empty \\
0,\,\,\,otherwise
\end{aligned}
\right.
\end{equation} 
$$
不同的路径是从道路网络中检索的，因此在下文中被称为路线。该路由定义的示例如图2所示
![fig2](/Decision/elements/BMW_fig2.png "fig2")    
考虑到其他汽车运动的不确定性，自动驾驶汽车必须不断选择最佳加速度a*，以最大化预期的累积贴现未来奖励：
$$
\begin{equation}
a^*_0 := \argmax_{a_0} E \left[
\sum^{\infty}_{\tau = 0} \gamma^\tau R^{t_{\tau+1}}|a^{t_\tau}_0
\right]
\end{equation}
$$
奖励应该包括惩罚碰撞、总加速度（提供舒适度）以及与交通法和基于曲率的参考速度的偏差。
由于所使用的自适应置信树 (ABT) 算法对多个片段进行采样来近似解决方案，因此不需要明确指定 POMDP 的模型属性（例如概率分布），而是将其作为生成模型

### Partially Observable Markov Decision Process
POMDP由 $\left<\mathcal{X,A,T,O,Z,R},b_0,\gamma\right>$