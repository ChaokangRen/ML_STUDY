## The lane-changing model MOBIL
大多数时间连续的微观单车道交通模型将单个“车辆驾驶员单元”$\alpha$的运动描述为其自身速度 $v_{\alpha}$ 的函数,到前车 $(\alpha-1)$的距离(bumper-to-bumper)用 $s_{\alpha}$表示，相对速度表示为 $\Delta v_{\alpha} = v_{\alpha} - v_{\alpha-1}$.  car-following模型的加速度表示为：
$$
a_{\alpha}:=\frac{dv_{alpha}}{dt}=a(s_{\alpha},v_{alpha},\Delta_{alpha})
$$
具体的车道变换，例如如图1所示从中央车道到中间车道，通常分别取决于当前车道和目标车道上的两辆跟随车辆。  
![lane-change](/Decision/elements/mobil_figure1.png "lane-change") 
考虑向左侧变道时，离中心车辆c的最近的车辆，用n和o分别表示新的和旧的后车，可能的变化后的加速度用 $\tilde a_c$ 表示。  
### Safety criterion
和其他换道模型一样，需要区分变道激励和安全约束。安全准则通过考虑对目标车道的上游车辆的影响来检车执行变道的可能性。我们的安全标准根据纵向加速度制定，保证在变道之后，successor的减速度不应该超过给定的安全减速度：
$$
\tilde a_n \geq -b_{safe}
$$
虽然表述为简单的不等式，但该条件包含纵向跟车模型通过加速度提供的所有信息，该加速度通常取决于间隙、速度，并最终取决于接近率，参见(1).特别是，如果纵向模型对速度差异具有内在的敏感性，则这种基本依赖性将转移到车道变换决策中。这样，如果后车速度快于自身速度，则目标车道中的后车与自身位置之间需要有较大的间隙才能满足安全约束。相反，如果后面的车辆较慢，则允许较小的间隙值。 与传统的间隙接受模型相比，这种方法仅通过对纵向加速度的依赖来间接依赖于间隙。 根据加速度对情况进行评估可以得出紧凑的公式。  
对于实际纵向模型来说，$b_safe$应该低于最大可能的加速度 $b_{max}$,在干燥路面上通常是$9m/s^2$.请注意，即使在完全自私的驾驶员的情况下，只要最大安全减速度 $b_{safe}$ 的值不大于基础纵向模型的最大可能减速度$b_{max}$，也可以防止事故发生。增加 $b_{safe}$ 的值通常会因个别车道变更而导致更强的扰动。 但目标车道上的跟随者的制动反应始终受到$b_{safe}$ 值的限制。 这在交通模拟中是相关的，因为执行车道变换意味着新跟随者的加速函数的输入参数的不连续变化。

### Incentive criterion for symmetric(US) lane change rules
激励标准通常确定车道变换是否改善了驾驶员的个人局部交通状况。礼貌因子 p 决定了这些车辆对换道决策的影响程度。对于对称超车规则，我们忽略车道之间的差异，并为车辆 c 驾驶员的换道决策提出以下激励条件：
$$
\tilde a_c - a_c + p(\tilde a_n - a_n + \tilde a_o - a_o) > \Delta a_{th}
$$
方程1右侧的切换阈值$\Delta a_{th}$模拟了一定的惯性，如果与“保持车道”指令相比整体优势微乎其微，则可以防止车道变换。综上所述，如果自身优势（加速增益）高于新老继任者的劣势（加速损失）与阈值$\Delta a_{th}$的加权和，则满足激励标准.请注意，阈值影响全局换道行为，而礼貌参数则根据所涉及的邻居影响局部换道行为。对于每个方向超过两条车道的交通的概括很简单。 如果对于中间车道上的车辆，相邻两个车道都满足激励标准，则变换到激励较大的车道。由于其他驾驶员的劣势和自身优势通过礼貌因子 p 进行平衡，因此换道模型包含经典博弈论的典型策略特征。  
p的值可以解释为利他主义的程度。它可以从 p = 0（对于自私的换道者）到 p > 1（对于无私的司机），对于无私的司机，如果换道导致followers的交通情况变糟糕，则不会考虑换道，反而如果可以提高followers的效率，即使存在不利因素也会变道。通过设置 p < 0，甚至可以对恶意驾驶员进行建模，他们接受自己的劣势以阻止他人。  
在特殊case中 p = 1且 $\Delta a_{th} = 0$,激励准则简化为：
$$
\tilde a_c + \tilde a_n + \tilde a_o > a_c + a_n + a_o
$$
因此，只有当所有相关车辆的加速度总和增加时，才会执行变道，这符合理想意义上的“最小化变道引起的整体制动”（MOBIL）的概念。在这种情况下，不需要额外的安全约束，因为只要加速度方面的优势低于制动减速度方面的劣势，方程（4）就会自动排除为了避免事故而进行的制动操作。因此，对应于 p = 1 的“理想”MOBIL 策略没有自由参数，因此可能被视为换道决策的“最小模型”。 在第 3 节中，我们研究了车道变更率（每公里和每小时），该比率主要由礼貌因素 p 决定。

### Application to multi-lane traffic simulations
