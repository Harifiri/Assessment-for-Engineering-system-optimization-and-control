一、交叉口内多车路权协同规划推导原理及规则说明：
1.划分两个范围，内圈范围为交叉口冲突区，外圈范围为跟车会车或超车区
2.当前路权进入内圈范围后，不再根据改变轨迹，在内侧与外侧范围内的车辆均可重规划
3.若优先级高的车辆进入规划区后，将从当前时刻开始，将所有可以重新规划的车辆的轨迹抹除，根据新优先级重新规划，由于同一方向来的车类别肯定相同（轻载或重载），因此不需要考虑超车或会车问题
4.路权的规划借鉴采用百度Apollo的轨迹规划方法，通过动态规划(DP)求出路权的凸空间再通过二次规划(QP)求出带有路权信息的预测最优轨迹，并根据预测的轨迹下放路权。为适应路权规划求解而做出的改进主要分为两方面：①在动态规划和二次规划中加入了动态路权的约束函数(由于动态路权为二次项，因此是凸问题，可以通过凸优化求解)②解除了固定终点的约束，为了激励车辆能够更快的通过交叉口(即在固定时间内通过更远的距离)，将终点位置加入目标函数中
5.上述的动态规划与二次规划求解空间在维护的ST图中提取，其中S为交叉口内沿参考线的路程，T为时间，ST图中的曲线即有了空间(S)与时间(T)的规划与预订能力，且通过对ST图分别求一阶导、二阶导、三阶导即可得到对应轨迹的速度、加速度、加加速度信息
5.车辆i在t时刻经过某一为之后，在一段时间dt内相同位置不允许有其他车辆通过，方法是将车辆在t时刻的ST图投影映射至整体ST图的[t, t+dt]影像上，这样其他车辆在规划时会认为ST图的[t, t+dt]有车辆i的障碍物，从而在规划时从时间和空间上进行路权的主动避让
6.优先级的具体计算规则：①普通车辆和普通车辆之间-FCFS②普通车辆和重载车辆-设置时间阈值，若普通车辆比重载车辆提前进入交叉口的时间大于此时间阈值，则普通车辆优先，否则重载优先
7.死锁问题解决方案：规划出车辆可以行驶至最远处的路权，然后在最后两个时刻的路权处停车，路权前端为最后两个路权处的最远距离

二、相关文件说明：
1.<waytous-multi-vehicle-planning\code文件夹>：主要代码
2.<waytous-multi-vehicle-planning\Ipopt-3.11.1-win64-intel13.1文件夹>：IPOPT开源求解器
3.<waytous-multi-vehicle-planning\RefPath.npy文件>：参考线信息

三、<waytous-multi-vehicle-planning\code文件夹>中各部分重要代码说明：
1.<main.py>：
	class IntersectionManage：自动交叉口管理最上游接口
		方法1：planner(self, t, car_info_=None)：
			@param: ①时间 ②进入交叉口控制域的车辆信息
			@function: 每2s调用一次用以进行交叉口内所有车辆的路权规划
			@return: None

		方法2：__update_state_list__(self, t, car_info_)：
			@param: ①时间 ②进入交叉口控制域的车辆信息
			@function: 用以更新交叉口管理系统的状态信息
			@return: None

		方法3：__set_priority__(self,t_):
			@param: ①时间
			@function: 每次有新车辆进入自动交叉口管理区域时调用用以更新优先级顺序
			@return: 更新后的车辆优先级列表


2.<param.py>：包括'地图'，'车辆固定参数(车长、车宽、最大\最小速度、最大\最小加速度、最大\最小加加速度)'、'路权范围参数'、'路权更新时间'、'投影至ST图上的延长时间'等具体参数信息(相应参数都已注释)
	         

3.<ST_graph.py>：
	class STGraph:：ST图管理类
		方法1：update_path(self, road_right, dir_)：
			@param: ①路权轨迹信息 ②路权轨迹相对应的车道编号
			@function: 用以更新规划好的路权ST图
			@return: None

		方法2：def purge_path(self, road_right, dir_):
			@param: ①路权轨迹信息 ②路权轨迹相对应的车道编号
			@function: 每次重规划前用以擦除当前时刻向后时间的ST图信息
			@return: None


4.<point.py>：
	class Point: 
		方法1：__init__(self, dir_, s_, t_, parent_=None):
			@param: ①车道方向 ②状态信息[s, vel, acc] ③结点对应时间 ④父节点
			@function: 初始化结点
			@return: None

		方法2：CalTotalCost(self, p_last):
			@param: ①父节点信息
			@function: 动态规划(DP)过程中计算当前结点代价值
			@return: 当前结点代价值

		方法3：Calculate_s_Bound(self):
			@param: None
			@function: 计算当前结点对应路权所占区域信息
			@return: 结点对应路权的前、后端参考新位置


5.<planner.py>：
	class Planner:
		方法1：SolveTrajectory(self, graph, point_, target_s):
			@param: ①全局ST图 ②初始结点信息 ③目标位置
			@function: 求解到达目标位置的路权轨迹
			@return: 到达目标位置的路权轨迹

		方法2：__DP__(self):
			@param: None
			@function: 求解到达目标位置的粗解(凸空间)
			@return: SuccessFlag, DP_Path(若能够成功求出到达目标位置的凸空间，则SuccessFlag置为True，否则SuccessFlag置为False)

		方法3：__QP__(self, DP_path, task_flag):
			@param: ①动态规划求解出的DP_path ②动态规划求解出的flag
			@function: 若task_flag为True，则求解通过交叉口的路权轨迹信息，否则求解进行死锁处理后的路权轨迹
			@return: 求解的路权轨迹信息


5.<coord_transform.py>：Frenet坐标与笛卡尔坐标系转换相关代码


6.<visual.py>:
	方法1：DrawTrajectory(trajectory_list, road_right_list, t_terminal):
		@param: ①QP求解的所有车辆预测轨迹列表 ②所有车辆路权轨迹列表 ③仿真时间
		@function: 基于Pygame的可视化代码
		@return: None

	方法2：DrawBackground(screen):
		@param: ①所在显示区域
		@function: 在screen上画出背景地图
		@return: None

	方法3：def DrawPointMessage(screen, point, road_right):
		@param: ①预测轨迹结点 ②路权结点信息
		@function: 在screen上画出车辆的路权所在位置
		@return: None
