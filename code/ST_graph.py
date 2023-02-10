from param import *
from point import Point

# np.empty((len(Reference), 1, min(len(Reference[0]), len(Reference[1]))), dtype=bool)
class STGraph:
    def __init__(self, lane_num, len_s):
        self.graph_len = len_s
        self.graph_step = np.zeros((lane_num, 1, self.graph_len), dtype=int)
        self.graph = np.zeros((lane_num, 1, self.graph_len), dtype=int)
        self.graph_step.fill(0)
        self.graph.fill(0)
        for i in range(200):
            self.graph = np.append(self.graph, self.graph_step, axis=1)

    def purge_path(self, road_right, dir_):
        if int((road_right[0][0] + 0.0001)/t_step) + len(road_right) > len(self.graph[dir_]):
            delta_t = int((road_right[0][0] + 0.0001)/t_step) + len(road_right) - len(self.graph[dir_]) + delta_ST_index
            for t in range(delta_t):
                self.graph = np.append(self.graph, self.graph_step, axis=1)

        for index in range(0, len(road_right), delta_update_index):
            index_t = int((road_right[index][0]+0.001)/t_step)

            index_sf = int(road_right[index][1][1]/s_step)
            index_sf = min(index_sf, self.graph_len)
            index_sb = int(road_right[index][1][0]/s_step)
            index_sb = max(0, index_sb)

            for i in range(index_t, index_t+delta_ST_index):
                self.graph[dir_][i][index_sb:index_sf] = 0

            if dir_ % 2 == 0:
                for index_ in [1, 2, 3, 4]:
                    y0, y1 = index_of_conflict[0][index_]
                    index_s0, index_s1 = self.get_intersection_of_set(index_sb, index_sf, y0, y1)

                    if index_s0 == -1:
                        continue

                    dir_index = (dir_+index_) % 6
                    if index_ == 1:
                        for i in range(index_t, index_t+delta_ST_index):
                            self.graph[dir_index][i][index_s0:index_s1] = 0
                    elif index_ == 2:
                        index_s0 = 1991 - index_s0
                        index_s1 = 1991 - index_s1
                        for i in range(index_t, index_t+delta_ST_index):
                            self.graph[dir_index][i][index_s1:index_s0] = 0
                    elif index_ == 3:
                        index_s0 = index_of_conflict[1][1][0] + (index_s0 - index_of_conflict[0][3][0])
                        index_s1 = index_of_conflict[1][1][0] + (index_s1 - index_of_conflict[0][3][0])
                        for i in range(index_t, index_t+delta_ST_index):
                            self.graph[dir_index][i][index_s0:index_s1] = 0
                    elif index_ == 4:
                        index_s0 = 1991 - index_s0
                        index_s1 = 1991 - index_s1
                        for i in range(index_t, index_t+delta_ST_index):
                            self.graph[dir_index][i][index_s1:index_s0] = 0
            else:
                for index_ in [-1, 1]:
                    y0, y1 = index_of_conflict[1][index_]
                    index_s0, index_s1 = self.get_intersection_of_set(index_sb, index_sf, y0, y1)

                    if index_s0 == -1:
                        continue

                    dir_index = (dir_ + index_) % 6
                    for i in range(index_t, index_t+delta_ST_index):
                        self.graph[dir_index][i][index_s0:index_s1] = 0

    def update_path(self, road_right, dir_):
        delta_t = int((road_right[0][0] + 0.0001)/t_step) + len(road_right) - len(self.graph[dir_]) + delta_ST_index
        if delta_t > 0:
            for t in range(delta_t):
                self.graph = np.append(self.graph, self.graph_step, axis=1)

        for index in range(0, len(road_right), delta_update_index):
            index_t = int((road_right[index][0]+0.001)/t_step)

            index_sf = int(road_right[index][1][1]/s_step)
            index_sf = min(index_sf, self.graph_len)
            index_sb = int(road_right[index][1][0]/s_step)
            index_sb = max(0, index_sb)

            for i in range(index_t, index_t+delta_ST_index):
                self.graph[dir_][i][index_sb:index_sf] = dir_+1

            if dir_ % 2 == 0:
                for index_ in [1, 2, 3, 4]:
                    y0, y1 = index_of_conflict[0][index_]
                    index_s0, index_s1 = self.get_intersection_of_set(index_sb, index_sf, y0, y1)

                    if index_s0 == -1:
                        continue

                    dir_index = (dir_+index_) % 6
                    if index_ == 1:
                        for i in range(index_t, index_t+delta_ST_index):
                            self.graph[dir_index][i][index_s0:index_s1] = dir_+1
                    elif index_ == 2:
                        index_s0 = 1991 - index_s0
                        index_s1 = 1991 - index_s1
                        for i in range(index_t, index_t+delta_ST_index):
                            self.graph[dir_index][i][index_s1:index_s0] = dir_+1
                    elif index_ == 3:
                        index_s0 = index_of_conflict[1][1][0] + (index_s0 - index_of_conflict[0][3][0])
                        index_s1 = index_of_conflict[1][1][0] + (index_s1 - index_of_conflict[0][3][0])
                        for i in range(index_t, index_t+delta_ST_index):
                            self.graph[dir_index][i][index_s0:index_s1] = dir_+1
                    elif index_ == 4:
                        index_s0 = 1991 - index_s0
                        index_s1 = max(940,1991 - index_s1)
                        for i in range(index_t, index_t+delta_ST_index):
                            self.graph[dir_index][i][index_s1:index_s0] = dir_+1
            else:
                for index_ in [-1, 1]:
                    y0, y1 = index_of_conflict[1][index_]
                    index_s0, index_s1 = self.get_intersection_of_set(index_sb, index_sf, y0, y1)

                    if index_s0 == -1:
                        continue

                    dir_index = (dir_ + index_) % 6
                    for i in range(index_t, index_t+delta_ST_index):
                        self.graph[dir_index][i][index_s0:index_s1] = dir_+1

    def get_intersection_of_set(self, x0, x1, y0, y1):
        if x0 > y1 or x1 < y0:
            return -1, -1

        return max(x0, y0), min(x1, y1)
