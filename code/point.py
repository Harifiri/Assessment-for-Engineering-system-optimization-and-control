import sys
from param import *
import math

class Point:
    def __init__(self, dir_, s_, t_, parent_=None):
        # dir~[6], s=[s,s',s"]
        self.dir = dir_
        self.s = s_
        self.t = t_

        self.sf = 0
        self.sb = 0

        self.parent = parent_

        self.t_cost = 0
        self.s_cost = 0
        self.obs_cost = 0
        self.vel_cost = 0
        self.acc_cost = 0
        self.jerk_cost = 0
        self.cost = sys.maxsize

    def CalTotalCost(self, p_last):
        return p_last.cost + self.t_cost + self.s_cost + self.obs_cost + self.vel_cost + self.acc_cost + self.jerk_cost

    def Calculate_s_Bound(self):
        self.sf = self.s[0] + car_lf + Mf
        self.sb = self.s[0] - car_lb - Mb
        return self.sf, self.sb

    def __del__(self):
        pass

