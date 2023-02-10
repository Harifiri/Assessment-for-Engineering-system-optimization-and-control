#!/usr/bin/python
# -*- coding: UTF-8 -*-
import math
from param import *
from point import Point
from ST_graph import STGraph
from planner import Planner
from visual import DrawTrajectory
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
import matplotlib as mpl
import matplotlib.patches as mpatches

# [dir, [s, ds, dds], t_in, car_id]
car_flow = [
    [1, [s_offset_0_, 5.0, 0], 0, 1],
    [2, [s_offset_0_, 6.5, 0], 2, 2],
    [4, [s_offset_0_, 7.5, 0], 6, 3],
    [0, [s_offset_0_, 8.5, 0], 10, 4],
    [5, [s_offset_0_, 5.0, 0], 16, 6],
    [3, [s_offset_0_, 5.0, 0], 20, 5],
]

class IntersectionManage(STGraph, Planner):
    def __init__(self, lane_num, len_s):
        STGraph.__init__(self, lane_num, len_s)
        Planner.__init__(self)

        self.flex_path_list = []
        self.flex_road_right_list = []
        self.flex_priority_list = []
        self.flex_flow_list = []
        self.flex_t_in_list = []
        self.flex_flag_list = []
        self.flex_car_num = 0

        self.fixed_path_list = []
        self.fixed_road_right_list = []
        self.fixed_car_num = 0

        self.sim_path_list = []
        self.sim_road_right_list = []
        self.sim_t_terminal = 0

        self.w_dir_of_priority = 0.5  # 0-1

    def planner(self, t, car_info_=None):
        # update time and car info
        self.__update_state_list__(t, car_info_)

        # if a vehicle entering the control zone at this time, calculate the priority
        if car_info_ != None:
            self.__set_priority__(t)

        # plan trajectory by priority
        for i in range(self.flex_car_num):
            if self.flex_flag_list[i] != True and self.flex_path_list[i] != []:
                time_index = int((t - self.flex_path_list[i][0].t + 0.001)/t_step)
                self.flex_flow_list[i][1] = self.flex_path_list[i][time_index].s
                self.flex_flow_list[i][2] = t
                # purge ST graph
                self.purge_path(self.flex_road_right_list[i][time_index:], self.flex_flow_list[i][0])
                self.flex_path_list[i] = self.flex_path_list[i][:time_index]
                self.flex_road_right_list[i] = self.flex_road_right_list[i][:time_index]

        for priority in range(self.flex_car_num):
            index = -1
            for i in range(self.flex_car_num):
                if self.flex_priority_list[i] == priority:
                    index = i
                    break

            if self.flex_flag_list[index]:
                continue

            car_info_temp = self.flex_flow_list[index]
            point_temp = Point(car_info_temp[0], car_info_temp[1], car_info_temp[2], parent_=None)
            
            if blocking_state[point_temp.dir] and self.flex_path_list[index]:
                continue

            if not blocking_state[point_temp.dir]:
                flex_path, flex_road_right, success_flag = self.SolveTrajectory(self.graph, point_temp, s_offset_2_[point_temp.dir%2])
            else:
                if not self.flex_path_list[index]:
                    flex_path, flex_road_right, success_flag = self.SolveTrajectory(self.graph, point_temp, s_offset_2_[point_temp.dir%2])
                    blocking_s[point_temp.dir] = flex_path[-1].s[0]

            if self.flex_path_list[index]:
                time_index = int((t - self.flex_path_list[index][0].t + 0.001)/t_step)
                self.flex_path_list[index] = self.flex_path_list[index][:time_index] + flex_path
                self.flex_road_right_list[index] = self.flex_road_right_list[index][:time_index] + flex_road_right
            else:
                self.flex_path_list[index] = flex_path
                self.flex_road_right_list[index] = flex_road_right
            self.update_path(flex_road_right, car_info_temp[0])
            self.flex_flag_list[index] = success_flag

            self.sim_path_list = self.flex_path_list + self.fixed_path_list
            self.sim_road_right_list = self.flex_road_right_list + self.fixed_road_right_list
            self.sim_t_terminal = max(self.sim_t_terminal, flex_path[-1].t)

            # t_list = []
            # s_list = []
            # v_list = []
            # acc_list = []
            # for p in flex_path:
            #     t_list.append(p.t)
            #     s_list.append(p.s[0])
            #     v_list.append(p.s[1])
            #     acc_list.append(p.s[2])
            #     print(p.t, p.s)
            #
            # plt.subplot(2, 1, 1)
            # fig = plt.figure()
            # # ax = fig.add_subplot(111)
            # # fig.add_axes(ax)
            #
            # ax1 = plt.subplot2grid((3, 1), (0, 0), colspan=1)
            # plt.plot(t_list, s_list, linewidth=1.0, color='black')
            # plt.xlabel("t")
            # plt.ylabel("s")
            # # fig.add_axes(ax1)
            #
            # ax2 = plt.subplot2grid((3, 1), (1, 0), colspan=1)
            # plt.plot(t_list, v_list, linewidth=1.0, color='black')
            # plt.xlabel("t")
            # plt.ylabel("v")
            # # fig.add_axes(ax2)
            #
            # ax3 = plt.subplot2grid((3, 1), (2, 0), colspan=1)
            # plt.plot(t_list, acc_list, linewidth=1.0, color='black')
            # plt.xlabel("t")
            # plt.ylabel("a")
            # # fig.add_axes(ax3)
            #
            # plt.show()


            # graph = np.transpose(self.graph[car_info_[0]])
            # plt.imshow(graph, cmap='binary', origin='lower')
            # plt.show()

    def __update_state_list__(self, t, car_info_):
        new_flex_path_list = []
        new_flex_road_right_list = []
        new_flex_priority_list = []
        new_flex_flow_list = []
        new_flex_t_in_list = []
        new_flex_flag_list = []
        new_flex_car_num = 0

        new_fixed_path_list = []
        new_fixed_road_right_list = []
        new_fixed_car_num = 0

        for i in range(self.flex_car_num):
            time_index = int((t-self.flex_path_list[i][0].t + 0.001)/t_step)

            if time_index < len(self.flex_path_list[i]):
                if self.flex_path_list[i][time_index].s[0] < s_offset_1_:
                    new_flex_path_list.append(self.flex_path_list[i])
                    new_flex_road_right_list.append(self.flex_road_right_list[i])
                    new_flex_priority_list.append(self.flex_priority_list[i])
                    new_flex_flow_list.append(self.flex_flow_list[i])
                    new_flex_t_in_list.append(self.flex_t_in_list[i])
                    new_flex_flag_list.append(self.flex_flag_list[i])
                    new_flex_car_num += 1
                else:
                    new_fixed_path_list.append(self.flex_path_list[i])
                    new_fixed_road_right_list.append(self.fixed_road_right_list[i])
                    new_fixed_car_num += 1

        for i in range(len(self.fixed_path_list)):
            new_fixed_path_list.append(self.fixed_path_list[i])
            new_fixed_road_right_list.append(self.fixed_road_right_list[i])
            new_fixed_car_num += 1
            # time_index = int((t - self.fixed_path_list[i][0].t + 0.001)/t_step)
            #
            # if time_index < len(self.fixed_path_list[i]):
            #     new_fixed_path_list.append(self.fixed_path_list[i])
            #     new_fixed_car_num += 1

        self.flex_path_list = new_flex_path_list
        self.fixed_road_right_list = new_flex_road_right_list
        self.flex_priority_list = new_flex_priority_list
        self.flex_flow_list = new_flex_flow_list
        self.flex_t_in_list = new_flex_t_in_list
        self.flex_flag_list = new_flex_flag_list
        self.flex_car_num = new_flex_car_num

        self.fixed_path_list = new_fixed_path_list
        self.fixed_road_right_list = new_fixed_road_right_list
        self.fixed_car_num = new_fixed_car_num

        if car_info_ != None:
            self.flex_path_list.append([])
            self.flex_road_right_list.append([])
            self.flex_priority_list.append(-1)
            self.flex_flow_list.append(car_info_)
            self.flex_t_in_list.append(t)
            self.flex_flag_list.append(False)
            self.flex_car_num += 1

    def __set_priority__(self,t_):
        max_wait_time = 10
        # calculate car priority
        # FCFS
        temp_flex_priority_list = [0 for i in range(self.flex_car_num)]
        temp_priority_cost_list = []

        for i in range(0, self.flex_car_num):
            temp_priority_cost = self.flex_t_in_list[i] - t_
            temp_priority_cost_list.append(temp_priority_cost)
            # temp_priority_cost_list.append(self.flex_t_in_list[i])
            for j in range(i):
                if temp_priority_cost_list[j] > temp_priority_cost_list[i]:
                    temp_flex_priority_list[j] += 1
                else:
                    temp_flex_priority_list[i] += 1

        self.flex_priority_list = temp_flex_priority_list

        print('cur priority list:',self.flex_priority_list)

        for i in range(self.flex_car_num):
            if self.flex_priority_list[i] > self.flex_priority_list[-1]:
                self.flex_flag_list[i] = False
        print('cur flag list:', self.flex_flag_list)


def FinishCheck(list):
    for flag in list:
        if not flag:
            return False
    return True


if __name__ == '__main__':
    init_RefLine()
    Manager = IntersectionManage(12, 2000)

    car_count = 0
    time_count = 0

    while True:
        sim_time = time_count*t_update
        print('sim time:', round(sim_time, 2))

        if car_count < len(car_flow):
            if abs(sim_time - car_flow[car_count][2]) < 0.01:
                Manager.planner(car_flow[car_count][2], car_info_=car_flow[car_count])
                car_count += 1
            else:
                Manager.planner(round(sim_time, 1))
        else:
            Manager.planner(round(sim_time, 1))

        time_count += 1

        if car_count == len(car_flow):
            print(Manager.flex_flag_list)
            if FinishCheck(Manager.flex_flag_list):
                break

        if sim_time > 500:
            break
    # for car_info in car_flow:
    #     Manager.planner(car_info[2], car_info_=car_info)

    colors_RGBA = np.array([ [99,178,238,255],
                    [118,218,145,255],
                    [248,203,127,255],
                    [248,149,136,255],
                    [124,214,207,255],
                    [145,146,171,255]])/255

    # for i in range(6):
    #     graph = np.transpose(Manager.graph[i])
    #
    #     image = [[[255, 255, 255] for j_ in range(len(graph[0]))] for i_ in range(len(graph))]
    #
    #     for i_ in range(len(graph)):
    #         for j_ in range(len(graph[0])):
    #             index = graph[i_][j_]
    #             image[i_][j_] = img_color[index]
    #
    #     time_list = np.arange(0, len(graph[0])*t_step, t_step)
    #
    #     # fig, ax = plt.subplots()
    #     # ax.set_xticks(time_list)
    #     image = np.array(image)
    #     im = plt.imshow(image, origin='lower')
    #     plt.xticks(np.arange(0, len(graph[0]), 1), time_list)
    #     plt.gca().xaxis.set_major_locator(MultipleLocator(100))
    #     plt.xlabel('T')
    #     plt.ylabel('S')
    #
    #     # colors_cmap = im.cmap(colors)
    #     # print(colors_cmap)
    #     # create a patch (proxy artist) for every color

    #     patches = [mpatches.Patch(color=colors_RGBA[i], label="Vehicle {l}".format(l=i)) for i in range(len(colors_RGBA))]
    #     # put those patched as legend-handles into the legend
    #     plt.legend(handles=patches, bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    #
    #     plt.show()

    # for i in range(6):
    #     t = []
    #     s = []
    #     v = []
    #     a = []
    #     cnt = 0
    #     text = "Vehicle {l}".format(l=i)
    #     for p in Manager.sim_path_list[i]:
    #         t.append(p.t)
    #         s.append(p.s[0])
    #         v.append(p.s[1])
    #         a.append(p.s[2])
    #     plt.plot(t, v, c=colors_RGBA[i], label=text)
    #     plt.xlabel('t')
    #     plt.ylabel('vel')
    #     plt.legend(loc = 'upper right')
    #
    # plt.show()
    #
    # for i in range(6):
    #     t = []
    #     s = []
    #     v = []
    #     a = []
    #     cnt = 0
    #     text = "Vehicle {l}".format(l=i)
    #     for p in Manager.sim_path_list[i]:
    #         t.append(p.t)
    #         s.append(p.s[0])
    #         v.append(p.s[1])
    #         a.append(p.s[2])
    #     plt.plot(t, a, c=colors_RGBA[i], label=text)
    #     plt.xlabel('t')
    #     plt.ylabel('acc')
    #     plt.legend(loc = 'upper right')
    #
    # plt.show()

    # for i in range(0, 100):
    #     print(Manager.sim_road_right_list[-1][i])
    DrawTrajectory(Manager.sim_path_list, Manager.sim_road_right_list, Manager.sim_t_terminal)
