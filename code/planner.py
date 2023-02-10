import sys
import time
from param import *
from point import Point
import pyomo.environ as pyo
from ST_graph import STGraph
import os

optimizers_path = os.path.join(os.path.abspath(os.path.dirname(os.getcwd())), r'Ipopt-3.11.1-win64-intel13.1\bin\ipopt')


num_of_points = 0
num_of_variables = 0
num_s_offset = 0
num_vel_offset = 0
num_acc_offset = 0
cur_dir = 0
s_task_ = 0

X_l = []
X_u = []
X_ref = []

H_matrix = np.zeros(0)
g_matrix = np.zeros(0)
C_obj = 0


def fb_x(model, i):
    return (X_l[i], X_u[i])


def obj_rule(model):
    X = np.array(model.X)
    X_t = np.transpose(X)

    return np.dot(np.dot(X_t, H_matrix), X) + np.dot(g_matrix, X) + C_obj


def kinematics_rule(model, i):
    if 0 < i < num_of_points:
        return model.X[i] == model.X[num_s_offset+i-1] + model.X[num_vel_offset+i-1]*t_step + t_step*t_step*model.X[num_acc_offset+i-1]/3 + t_step*t_step*model.X[num_acc_offset+i]/6
    elif num_vel_offset < i < num_acc_offset:
        return model.X[i] == model.X[num_s_offset+i-1] + model.X[num_of_points+i-1]*t_step/2 + model.X[num_of_points+i]*t_step/2

    return pyo.Constraint.Skip


def road_right_rule(model, i):
    if 0 < i < num_of_points:
        return model.X[i] + car_lf + Mf <= X_u[i]

    return pyo.Constraint.Skip


def target_rule(model, i):
    if i == num_of_points-1:
        return model.X[i] >= s_task_

    return pyo.Constraint.Skip


class Planner:
    def __init__(self):
        self.dir = 0
        self.start_state_ = [0, 0, 0]
        self.t_in_ = 0
        self.init_point = Point(self.dir, self.start_state_, 0)
        self.init_point.cost = 0
        self.ST = []
        self.t_terminal_ = 0
        self.s_terminal_ = 0

        # DP param
        self.dense_ds_DP = 1
        self.sparse_ds_DP = 5
        self.dense_s_terminal_ = 100
        self.sparse_s_terminal_ = 0
        self.dt_DP = 2
        self.w_obs_DP = 0.001
        self.w_t_DP = 10.0
        self.w_s_DP = 0.0
        self.w_vel_DP = 0.001
        self.w_acc_DP = 0.01
        self.w_jerk_DP = 0.25

        # QP param
        self.w_s_QP = 0.0
        self.w_s_ref_QP = 0.0001
        self.w_vel_QP = 0.001
        self.w_acc_QP = 0.005
        self.w_jerk_QP = 0.01

    def SolveTrajectory(self, graph, point_, target_s):
        self.dir = point_.dir
        self.start_state_ = point_.s
        self.t_in_ = point_.t
        self.init_point = Point(self.dir, self.start_state_, self.t_in_)
        self.init_point.cost = 0
        self.ST = graph[self.dir]
        self.t_terminal_ = 300
        self.s_terminal_ = target_s + 2*self.dt_DP*self.sparse_ds_DP
        self.sparse_s_terminal_ = self.s_terminal_ - self.dense_s_terminal_

        DP_res, task_flag = self.__DP__()
        QP_res = self.__QP__(DP_res, task_flag)

        return QP_res

    def __DP__(self):
        process_flag = False

        # Create cost table
        dimension_t_ = int((self.t_terminal_ + 0.01)/self.dt_DP)
        dense_dimension_s_ = int((self.dense_s_terminal_ + 0.1)/self.dense_ds_DP)
        sparse_dimension_s_ = int((self.sparse_s_terminal_ + 0.1)/self.sparse_ds_DP)
        CostTable = [[None for index_s_ in range(dense_dimension_s_ + sparse_dimension_s_)] for index_t_ in range(dimension_t_)]
        CostTable[0][0] = self.init_point

        row_limit = [[0, 0] for index_t_ in range(dimension_t_)]  # row limit table

        min_cost_list = [0, 0]

        # init cost table
        for index_t_ in range(1, dimension_t_):
            temp_t = self.init_point.t + index_t_*self.dt_DP
            temp_t = round(temp_t, 1)

            for index_s_ in range(0, dense_dimension_s_):
                temp_s = self.init_point.s[0] + index_s_*self.dense_ds_DP
                s_temp = [temp_s, 0, 0]
                CostTable[index_t_][index_s_] = Point(self.dir, s_temp, temp_t)

            for index_s_ in range(dense_dimension_s_, dense_dimension_s_ + sparse_dimension_s_):
                temp_s = self.init_point.s[0] + self.dense_s_terminal_ + (index_s_ - dense_dimension_s_)*self.sparse_ds_DP
                s_temp = [temp_s, 0, 0]
                CostTable[index_t_][index_s_] = Point(self.dir, s_temp, temp_t)
        # print(CostTable)
        # get row range
        last_vel_max = self.init_point.s[1]
        last_vel_min = self.init_point.s[1]
        last_s_max = 0
        last_s_min = 0
        for index_t_ in range(1, dimension_t_):
            temp_vel_max = min(vel_max, last_vel_max+self.dt_DP*acc_max)
            temp_s_max = last_s_max + temp_vel_max*self.dt_DP

            temp_vel_min = max(vel_min, last_vel_min + self.dt_DP*dec_min)
            temp_s_min = last_s_min + temp_vel_min*self.dt_DP

            if temp_s_max < self.dense_s_terminal_:
                index_s_max = int(temp_s_max/self.dense_ds_DP) + 1
            else:
                index_s_max = dense_dimension_s_ + int((temp_s_max - self.dense_s_terminal_)/self.sparse_ds_DP) + 1

            row_limit[index_t_][1] = min(index_s_max, dense_dimension_s_ + sparse_dimension_s_ - 1)

            if temp_s_min < self.dense_s_terminal_:
                index_s_min = int(temp_s_min/self.dense_ds_DP)
            else:
                index_s_min = dense_dimension_s_ + int((temp_s_min - self.dense_s_terminal_)/self.sparse_ds_DP)

            row_limit[index_t_][0] = max(0, index_s_min)

            last_vel_max = temp_vel_max
            last_vel_min = temp_vel_min
            last_s_max = temp_s_max
            last_s_min = temp_s_min

        # for list in row_limit:
        #     print(list)

        # Calculate cost and update parent node
        min_cost = sys.maxsize
        min_cost_i_t = 0
        min_cost_i_s = 0

        final_index_t_ = 0
        final_index_s_ = 0

        for index_t_ in range(1, len(CostTable)):
            row_min, row_max = row_limit[index_t_]
            row_min_last, row_max_last = row_limit[index_t_-1]

            if process_flag:
                break

            for index_s_ in range(row_min, row_max+1):
                p_temp_ = CostTable[index_t_][index_s_]

                if blocking_state[self.dir] and p_temp_.s[0] > blocking_s[self.dir]:
                    continue

                for index_s1_ in range(row_min_last, row_max_last+1):
                    p_last_ = CostTable[index_t_-1][index_s1_]
                    if p_last_.cost == sys.maxsize:
                        continue

                    # acc, vel, jerk cost
                    acc, p_temp_.acc_cost = self.__CalAccCost__(p_last_, p_temp_)
                    if p_temp_.acc_cost == sys.maxsize:
                        continue

                    vel, p_temp_.vel_cost = self.__CalVelCost__(acc, p_last_)
                    if p_temp_.vel_cost == sys.maxsize:
                        continue

                    p_temp_.jerk_cost = self.__CalJerkCost__(acc, p_last_)
                    if p_temp_.jerk_cost == sys.maxsize:
                        continue

                    # conflict cost
                    p_temp_.obs_cost = self.__CalObsCost__(p_temp_)
                    if p_temp_.obs_cost == sys.maxsize:
                        continue

                    p_temp_.t_cost = self.w_t_DP*math.pow(p_temp_.t, 2)
                    p_temp_.s_cost = self.w_s_DP/(1+math.pow(p_temp_.s[0], 2))

                    cost_temp = p_temp_.CalTotalCost(p_last_)

                    if cost_temp < p_temp_.cost:
                        p_temp_.s[1] = vel
                        p_temp_.s[2] = acc
                        p_temp_.cost = cost_temp
                        p_temp_.parent = p_last_

                        if index_s_ > final_index_s_ and p_temp_.s[0] < s_offset_1_:
                            final_index_t_ = index_t_
                            final_index_s_ = index_s_

                        if p_temp_.s[0] >= self.s_terminal_:
                            process_flag = True
                            if p_temp_.cost < min_cost:
                                min_cost = p_temp_.cost
                                min_cost_i_t = index_t_
                                min_cost_i_s = index_s_

        # backtracking
        res = []

        if process_flag:
            p = CostTable[min_cost_i_t][min_cost_i_s]
        else:
            p = CostTable[final_index_t_][final_index_s_]

        while p != None:
            res.append([p.s, p.t])
            p = p.parent

        res.reverse()
        # print('dynamic programming path:')
        # print(res)

        del CostTable

        return res, process_flag

    def __QP__(self, DP_path, task_flag):
        # if DP_path == False:
        #     return self.__emergency_braking__()

        global num_of_points
        global num_of_variables
        global num_s_offset
        global num_vel_offset
        global num_acc_offset
        global s_task_

        global X_l
        global X_u
        global X_ref

        global H_matrix
        global g_matrix
        global C_obj

        num_of_points = int((self.dt_DP+0.0001)/t_step)*(len(DP_path)-1) + 1
        num_of_variables = 3*num_of_points
        num_s_offset = 0
        num_vel_offset = num_of_points
        num_acc_offset = 2*num_of_points

        s_task_ = DP_path[-1][0][0]
        # print(DP_path[-1])

        X_ref, X_l, X_u = self.__gen_bound_list__(DP_path, task_flag)

        H_matrix, g_matrix, C_obj = self.__gen_obj_matrix__()

        model = pyo.ConcreteModel()  # 创建模型对象

        model.I = pyo.Set(initialize=[i for i in range(3 * num_of_points)])

        model.X = pyo.Var(model.I, within=pyo.Reals, bounds=fb_x)

        # define objective function
        # sense = minimize(Default) / maximize
        model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

        model.kinematics = pyo.Constraint(model.I, expr=kinematics_rule)
        model.road_right = pyo.Constraint(model.I, expr=road_right_rule)
        model.target = pyo.Constraint(model.I, expr=target_rule)

        solution = pyo.SolverFactory('ipopt', executable=optimizers_path).solve(model)
        solution.write()

        x_opt = np.array([pyo.value(model.X[i]) for i in model.I])

        opt_path = []
        opt_road_right = []
        for i in range(num_of_points):
            temp_ponit = Point(self.dir, [x_opt[i], x_opt[i+num_vel_offset], x_opt[i+num_acc_offset]], round(self.t_in_ + i*t_step, 2))
            temp_ponit.Calculate_s_Bound()
            opt_path.append(temp_ponit)

        path_len = num_of_points - num_of_points%delta_update_index

        for i in range(0, path_len, delta_update_index):
            sf_max = 0
            sb_min = sys.maxsize

            for j in range(i, i+delta_update_index):
                temp_ponit = opt_path[j]
                sf_max = max(sf_max, temp_ponit.sf)
                sb_min = min(sb_min, temp_ponit.sb)

            for j in range(i, i+delta_update_index):
                opt_road_right.append((opt_path[j].t, (sb_min, sf_max)))

        opt_path = opt_path[:path_len]
        opt_road_right = opt_road_right[:path_len]

        if not task_flag:
            brake_t = opt_path[-1].t
            brake_s = opt_path[-1].s[0]
            brake_vel = opt_path[-1].s[1]
            brake_time = int(brake_vel/abs(dec_min)) - int(brake_vel/abs(dec_min))%t_update + t_update
            brake_acc = brake_vel/brake_time
            brake_sf = opt_road_right[-1][-1][-1]

            brake_path = []
            brake_road_right = []
            for i in range(int((brake_time + 0.001)/t_step)):
                delta_t = t_step*(i+1)
                temp_vel = brake_vel - brake_acc*delta_t
                temp_s = brake_s + (brake_vel+temp_vel)*delta_t/2
                temp_ponit = Point(self.dir, [temp_s, temp_vel, brake_acc], round(brake_t+delta_t, 2))
                temp_ponit.Calculate_s_Bound()
                brake_path.append(temp_ponit)

            for i in range(0, len(brake_path), delta_update_index):
                sb_min = sys.maxsize

                for j in range(i, i + delta_update_index):
                    temp_ponit = brake_path[j]
                    sb_min = min(sb_min, temp_ponit.sb)

                for j in range(i, i + delta_update_index):
                    brake_road_right.append((brake_path[j].t, (sb_min, brake_sf)))

            opt_path = opt_path + brake_path
            opt_road_right = opt_road_right + brake_road_right

        return opt_path, opt_road_right, task_flag


    def __gen_bound_list__(self, path, task_flag):
        list_ref = [0 for i in range(num_of_points)]
        list_l = [ -1000 for i in range(num_of_variables) ]
        list_u = [ 10000 for i in range(num_of_variables) ]

        list_ref[0] = path[0][0][0]
        list_ref[-1] = path[-1][0][0]

        list_l[0] = path[0][0][0]
        list_u[0] = path[0][0][0]
        list_l[num_vel_offset] = path[0][0][1]
        list_u[num_vel_offset] = path[0][0][1]
        list_l[num_acc_offset] = path[0][0][2]
        list_u[num_acc_offset] = path[0][0][2]

        for i in range(len(path)-1):
            p0 = path[i]
            p1 = path[i+1]

            s0 = p0[0][0]
            s1 = p1[0][0]
            delta_s = s1 - s0

            index_t0 = int(p0[1]/t_step)

            temp_num_ = int(self.dt_DP/t_step + 0.1)

            for j in range(temp_num_):
                index_t = index_t0 + j
                index_t_list = i*temp_num_ + j

                if index_t_list == 0 or index_t_list == num_of_points-1:
                    pass

                s_temp = s0 + delta_s*j/temp_num_
                list_ref[index_t_list] = s_temp

                if index_t >= len(self.ST):
                    continue

                s_temp_int = int(s_temp)

                for index_s in range(s_temp_int, -1, -1):
                    if self.ST[index_t][index_s]:
                        list_l[index_t_list] = index_s*s_step + car_lb + Mb + 1
                        break

                for index_s in range(s_temp_int, len(self.ST[index_t])):
                    if self.ST[index_t][index_s]:
                        list_u[index_t_list] = index_s*s_step - 1
                        break

        # if task_flag != True:
        #     list_u[num_s_offset + num_of_points - 1] = s_offset_1_
        #     list_l[num_vel_offset + num_of_points - 1] = 0
        #     list_u[num_vel_offset + num_of_points - 1] = 0
        #     list_l[num_acc_offset + num_of_points - 1] = 0
        #     list_u[num_acc_offset + num_of_points - 1] = 0


        for i in range(num_vel_offset+1, num_acc_offset-1):
            list_l[i] = vel_min
            list_u[i] = vel_max

        for i in range(num_acc_offset+1, num_of_variables-1):
            list_l[i] = dec_min
            list_u[i] = acc_max

        return list_ref, list_l, list_u

    def __gen_obj_matrix__(self):
        Matrix_H = np.zeros((num_of_variables, num_of_variables))
        # s_ref cost
        for i in range(num_of_points):
            Matrix_H[i][i] += self.w_s_ref_QP
        # vel cost
        for i in range(num_vel_offset, num_acc_offset):
            Matrix_H[i][i] += self.w_vel_QP
        # acc cost
        for i in range(num_acc_offset, num_of_variables):
            Matrix_H[i][i] += self.w_acc_QP
        # jerk cost
        Matrix_H[num_acc_offset][num_acc_offset] += self.w_jerk_QP/t_step_squared
        Matrix_H[num_of_variables-1][num_of_variables-1] += self.w_jerk_QP/t_step_squared
        for i in range(num_acc_offset + 1, num_of_variables - 1):
            Matrix_H[i][i] += 2*self.w_jerk_QP/t_step_squared
        for i in range(num_acc_offset + 1, num_of_variables):
            Matrix_H[i][i-1] += -2*self.w_jerk_QP/t_step_squared

        # s_ref cost
        Matrix_g = np.zeros(num_of_variables)
        C_temp = 0

        Matrix_g[num_of_points-1] -= self.w_s_QP
        C_temp += self.w_s_QP*s_offset_0_
        for i in range(num_of_points):
            Matrix_g[i] = -2*self.w_s_ref_QP*X_ref[i]
            C_temp += self.w_s_ref_QP*X_ref[i]*X_ref[i]

        return Matrix_H, Matrix_g, C_temp

    def __CalAccCost__(self, last_p, cur_p):
        acc = 2*((cur_p.s[0]-last_p.s[0])/self.dt_DP - last_p.s[1])/self.dt_DP
        cost = 0
        if acc < dec_min or acc > acc_max:
            cost = sys.maxsize
        else:
            w_acc_squared = self.w_acc_DP*self.w_acc_DP
            cost = (self.w_acc_DP + w_acc_squared/(1+math.exp(acc_max-acc)) + w_acc_squared/(1+math.exp(acc-dec_min)))*math.pow(acc, 2)

        return acc, cost

    def __CalVelCost__(self, acc, last_p):
        vel = last_p.s[1] + acc*self.dt_DP
        if vel < vel_min or vel > vel_max:
            cost = sys.maxsize
        else:
            cost = 0

        return vel, cost

    def __CalJerkCost__(self, acc, last_p):
        jerk = (acc - last_p.s[2])/self.dt_DP

        if jerk < jerk_min or jerk > jerk_max:
            cost = sys.maxsize
        else:
            cost = self.w_jerk_DP*math.pow(jerk, 2)

        return cost

    def __CalObsCost__(self, cur_p):
        sf, sb = cur_p.Calculate_s_Bound()

        index_sf = int(sf/s_step)
        index_sb = int(sb/s_step)

        s_safe = 10
        index_safe = int(s_safe/s_step)

        cost = 0

        index_t = int(cur_p.t/t_step)

        if index_t < len(self.ST):
            for index_s_ in range(index_sb, index_sf+1):
                if self.ST[index_t][index_s_]:
                    cost = sys.maxsize
                    break
            if cost == 0:
                min_l = index_safe
                for index_s_ in range(max(0,index_sb-index_safe), index_sb):
                    if self.ST[index_t][index_s_]:
                        min_l = min(min_l, abs(index_s_ - index_sb))
                        break
                for index_s_ in range(index_sf, index_sf+index_safe):
                    if self.ST[index_t][index_s_]:
                        min_l = min(min_l, abs(index_s_ - index_sf))
                        break
                cost = self.w_obs_DP*(s_safe - min_l)*(s_safe - min_l)

        return cost

if __name__ == '__main__':

    ST = STGraph(12, int( max(s_offset_2_[0], s_offset_2_[0] ) ) + car_lf + Mf_max )

    s = [0, 7.5, 0]

    p = Point(0, s, 0)

    planner_ = Planner(ST.graph, p)

    trajectory = planner_.SolveTrajectory()
    # sf_QP = []
    # sb_QP = []
    # t_ = []
    #
    # t_DP = []
    # s_DP = []
    #
    # for p in DP_Path:
    #     t_DP.append(p[1])
    #     s_DP.append(p[0][0])
    #
    # for i in range(len(s_path)):
    #     t_.append(i*t_step)
    #     sb_QP.append(s_path[i] - car_lb - Mb)
    #     k_temp = (ds_path[i]/vel_max)*(ds_path[i]/vel_max)
    #     sf_QP.append(s_path[i]+car_lf+Mf_min+Mf_delta*k_temp)

    # plt.subplot(2, 1, 1)
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # fig.add_axes(ax)
    #
    # rect = plt.Rectangle((20, 0), 5, 100, color='yellow', alpha=1)
    # ax.add_patch(rect)
    # rect = plt.Rectangle((25, 200), 5, 100, color='yellow', alpha=1)
    # ax.add_patch(rect)
    #
    # t_ = t_[:-10]
    # s_path = s_path[:-10]
    # sb_QP = sb_QP[:-10]
    # sf_QP = sf_QP[:-10]
    # plt.plot(t_, s_path, color = "red",linewidth=1.0, label="QP result")
    # plt.plot(t_DP, s_DP, color="blue", linewidth=1.0, label="DP result")
    # plt.plot(t_, sb_QP, color="black", linewidth=1.0, label="Road right bound")
    # plt.plot(t_, sf_QP, color="black", linewidth=1.0)
    #
    # plt.xlabel("T")
    # plt.ylabel("S")
    #
    # plt.legend(loc='best')

    ##########################################

    # ax1 = plt.subplot2grid((3, 1), (0, 0), colspan=1)
    # plt.plot(t_[:-10], s_path[:-10], linewidth=1.0, color='black')
    # plt.xlabel("t")
    # plt.ylabel("s")
    #
    # ax2 = plt.subplot2grid((3, 1), (1, 0), colspan=1)
    # plt.plot(t_[:-10], ds_path[:-10], linewidth=1.0, color='black')
    # plt.xlabel("t")
    # plt.ylabel("v")
    #
    # ax3 = plt.subplot2grid((3, 1), (2, 0), colspan=1)
    # plt.plot(t_[:-10], dds_path[:-10], linewidth=1.0, color='black')
    # plt.xlabel("t")
    # plt.ylabel("a")
    #
    # plt.show()



