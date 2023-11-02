# Car-Following Models Based on Driving Strategies
## 1.Model Criteria
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