# Intelligent Driver Model

## Elementary Car-Following Models
 
distance headway = (distance) gap + length of the leading vehicle,  
(time) headway = time gap + occupancy time interval of the leading vehicle.

### Mathematical Description
![car-following-models](/Decision/elements/car-following_models.png "car-following")  
定义：  
$x_{\alpha}(t)$: 理解为在道路上的道路起点的相对位置  
$v_{\alpha}(t)$: 车速  
$l_{\alpha}$: 车长  
另外一些其他变量如加速度$\dot v = dv/dt$，以及刹车灯或者指示灯的亮灭。从车辆的位置和长度，可以得到车辆的间隙距离：  
$$
\begin{equation}
s_{\alpha} = x_{\alpha-1}-l_{\alpha-1} - x_{\alpha} = x_l - l_l - x_{\alpha}
\end{equation}
$$  
为了简便，有时会将$\alpha-1$写成$l$，后续章节会将间隙$s_{alpha}$表示为自车车速，前车位置和前车车速的函数。在连续时间模型中，驾驶员的响应直接以加速度函数$a_{mic}(s,v,v_l)给出，从而产生一组以下形式的耦合常微分方程:  
$$
\begin{equation}
\dot x_{\alpha}(t) = \frac{{\rm d}x_{\alpha}(t)}{{\rm d}t} = v_{\alpha}(t)
\end{equation}
$$
$$
\begin{equation}
\dot v_{\alpha}(t) = \frac{{\rm d}v_{\alpha}(t)}{{\rm d}t} = a_{mic}(s_{\alpha},v_{\alpha},v_l) = \tilde a_{mic}(s_{\alpha},v_{\alpha},\Delta v_{\alpha})
\end{equation}
$$ 
在大多数加速度函数中，大多数前车的车速仅以速度差的形式输入，即：
$$
\begin{equation}
\Delta v_{alpha} = v_{\alpha} - v_{\alpha -1}= v_{\alpha}  - v_l
\end{equation} 
$$
所以可以推导出：
$$
\begin{equation}
\tilde a_{mic}(s_{\alpha},v_{\alpha},\Delta v_{\alpha}) = a_{mic}(s_{\alpha},v_{\alpha},v_{\alpha}-\Delta v) 
\end{equation}
$$
式(1)也可以重新写为：
$$
\begin{equation}
\dot s_{alpha}(t) = \frac{{\rm d}s_{\alpha}(t)}{{\rm d}t} = v_l(t) - v_{\alpha}(t) = -\Delta v_{\alpha}(t)
\end{equation} 
$$
除此之外，还有离散时间跟车模型，其中时间不是建模为连续变量，而是离散为有限且通常恒定的时间步长，获得一般形式的迭代耦合图，而不是微分方程
$$
\begin{equation}
v_{\alpha}(t + \Delta t) = v_{mic}(s_{\alpha}(t),v_{\alpha}(t),v_l(t))
\end{equation} 
$$
$$
\begin{equation}
x_{\alpha}(t + \Delta t) = x_{\alpha}(t) + \frac{v_{\alpha}(t) +v_{\alpha}(t +\Delta t)}{2}\Delta t
\end{equation} 
$$
假设更新时间步长Δt恒定，一种简单但有效的显式更新方法是，在每一个时间周期内，给定加速度恒定的假设。即：

$$
\begin{equation}
v_{\alpha}(t+\Delta t) = v_{\alpha}(t) + a_{mic}(s_{\alpha}(t),v_{\alpha}(t),v_l(t)) \Delta t
\end{equation} 
$$
$$
\begin{equation}
x_{\alpha}(t+\Delta t) = x_{\alpha}(t) + \frac{v_{\alpha}(t) + v_{\alpha}(t + \Delta t)}{2}\Delta t
\end{equation} 
$$
如果设置以下方式，那么带ballistic更新方法的连续时间模型在数学上就等价与离散时间模型：
$$
\begin{equation}
a_{mic}(s,v,v_l)=\frac{v_{mic}(s,v,v_l)-v}{\Delta t}
\end{equation} 
$$

### Steady State Equilibrium and the Fundamental Diagram
状态平衡的意思是在真实道路环境的一种平衡，从技术上讲，这意味着所有驾驶员和车辆的模型参数都是相同的，即表征相应模型的加速度或速度函数不依赖于车辆内在参数。从建模的角度来看，稳态平衡具有以下两个条件：
* Homogeneous traffic:所有的车辆处于相同的车速($v_{\alpha}=v$),并且与前车保持相同的间隙(s_{\alpha}=s)
* No accelerations:$\dot v_{\alpha} = 0 or v_{\alpha}(t +\Delta t) = v_{\alpha}(t)$
对于具有$a_{mic}$或$\tilde a_{mic}(s,v,0)$形式的加速函数的时间连续模型，这意味着:  
$$
\begin{equation}
a_{mic}(s,v,v) = 0 \,\, or \,\, \tilde a_{mic}(s,v,0) = 0
\end{equation} 
$$
同时，条件
$$
\begin{equation}
v_{mic}(s,v,v) = v
\end{equation} 
$$
对式(7)的离散模型是有效的。根据模型的不同，微观稳态关系 (12) 或 (13) 可以求解
* 均衡速度$v_e(s)$,是对间隙的函数
* 均衡间隙$s_e(v)$,在给定速度下  

**Microscopic fundamental diagram**  
方程（12）和（13）允许可能的稳态的一维流形，可以用距离间隙s参数化，并用平衡速度函数$v_e(s)$描述，也称为微观基本图。
**Transition to macroscopic relations**为了获得距离间隙s和密度ρ之间的微观-宏观关系，我们直接定义密度为每条道路长度的车辆数量。对于一个给定的车长$l$,得到：
$$
\begin{equation}
s_{\alpha} = s = \frac{1}{\rho}-l
\end{equation} 
$$
此外，稳态平衡意味着所有车辆的速度相同并且等于宏观速度。
$$
\begin{equation}
V(x,t) = <v_{\alpha}(t)>=v_e(s)
\end{equation} 
$$
根据这些关系，我们可以推导出宏观稳态速度-密度图和宏观基本图:  
$$
\begin{equation}
V_e(\rho)=v_e(\frac{1}{\rho}-l),\,\,Q_e(\rho)=\rho v_e(\frac{1}{\rho} - l)
\end{equation} 
$$










---
## Car-Following Models Based on Driving Strategies
### 1.Model Criteria
与最小模型相比，编码驾驶行为的加速度或速度函数至少应该对以下方面进行建模:  
1. 加速度是速度的严格递减函数，而且，在没有其他车辆或者障碍物约束的情况下，车辆的加速是让车速趋近于期望速度的：  
$$
\begin{equation}
\frac{\partial a_{mic}(s,v,v_l)}{\partial v} < 0,
\lim_{s\rightarrow\infty}a_{mic}(s,v_0,v_l)=0 
\end{equation}
$$  
2. 加速度是对前车的距离s的递增函数：  

$$
\begin{equation}
\frac{\partial a_{mic}(s,v,v_l)}{\partial s} \geq 0,
\lim_{s\rightarrow\infty}a_{mic}(s,v_0,v_l)=0 
\end{equation}
$$
    成为等式的条件是其他车辆或者障碍物对自车不产生任何影响，定义为：  
$$
\begin{equation}
a_{free}(v)= \lim_{s\rightarrow \infty} a_{mic}(s,v,v_l)=\geq a_{mic}(s,v,v_l)
\end{equation}
$$
3. 加速度是前车速度的增函数。结合要求（1），这也意味着加速度随着接近前车（或障碍物）的速度而减小（减速度增大）：

$$
\begin{equation}
\frac{\partial \tilde a_{mic}(s,v,\Delta v)}{\partial \Delta v} \leq 0 or 
\frac{\partial \tilde a_{mic}(s,v,v_l)}{\partial \Delta v_l} \geq 0,
\frac{\lim_{s\rightarrow\infty}a_{mic}(s,v_0,v_l)}{\partial v_l}=0 

\end{equation}
$$

4. 于前车保持最小间隙，(保险杠到保险杠的距离)$s_0$(在静止时间也是如此)，如果由过去的事件导致间隙小于$s_0$，也不会重新回到$s_0$:
$$
\begin{equation}
a_{mic}(s,0,v_l) = 0 {\,\,}for {\,\,} all {\,\,}v_l\geq 0,s\leq s_0
\end{equation}
$$
满足这些要求的跟车模型是完整的，因为它可以一致地描述单车道交通中可能出现的所有情况。特别是，(i)所有车辆相互作用都是有限范围的,(ii)后面的车辆不会被“拖拽”，有:
$$
\begin{equation}
a_{mic}(s,v,v_l) \leq a_mic(\infty,v,v'_l) = a_{free}(v) \,\,\, for \,\,all\,\,\, s,v,v_l,and\, v'_l
\end{equation}
$$
(iii)均衡速度是存在的，它具有已经为最佳速度函数假设的属性  
$$
\begin{equation}
v'_e(s)\geq 0,\,\,\, v_e(0) = 0,\,\,\, \lim_{s\rightarrow \infty}v_e(s) = v_0
\end{equation}
$$
### Gipps' Model
