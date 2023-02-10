import os
import math
import numpy as np
from coord_transform import frenet_to_cartesian1D

path_ = os.path.join(os.path.dirname(os.getcwd()), 'RefPath.npy')


w = 1600
h = 1200
center_pos = (w/2, h/2)

inner_r = 250
outer_r = 600

#  926, 976  for dir0-2
#  1065, 1015 for dir0-2
# 939, 939 for dir 0-1

t_step = 0.25
t_step_squared = t_step*t_step
s_step = 1  # m
t_current = 0

t_delay = 0
t_update = 0.5
delta_delay_index = int((t_delay + 0.001)/t_step)
delta_update_index = int((t_update + 0.001)/t_step)
delta_ST_index = delta_update_index

t_max = 0

s_offset_0_ = 351
s_offset_1_ = 751

s_offset_2_ = [s_offset_1_+492+400, s_offset_1_+463+400]

dir_priority_cost_table = [1, 1, 0, 0, 0, 0, 0]

blocking_state = [False, False, False, False, False, False]
blocking_s = [s_offset_1_, s_offset_1_, s_offset_1_, s_offset_1_, s_offset_1_, s_offset_1_]

index_of_conflict = [
    [
        [],
        # 0-1
        [0, int(939/s_step)],
        # 0-2
        [int(926/s_step), int(976/s_step)],
        # 0-3
        [int((1994-939-1)/s_step), int((1994-1)/s_step)],
        # 0-4
        [int(1015/s_step), int(1065/s_step)],
        # 0-5
        [],
    ],
    [
        # 1-1
        [],
        #1-2
        [int((1965-939-1)/s_step), int((1965-1)/s_step)],
        # 1-0
        [0, int(939/s_step)],
    ],
]

conflict_s0 = 0
conflict_s1 = 0
conflict_s2 = 0

# Car Param
car_w = 10  # vehicle width (m)
car_lf = 10  # vehicle front overhang length(m)
car_lb = 5  # vehicle rear overhang length(m)
vel_max = 10  # (m/s)
vel_min = 0  # (m/s)
acc_max = 2.0  # (m/s^2)
dec_min = -2.0
jerk_max = 0.5  # (m/s^3)
jerk_min = -0.5  # (m/s^3)
# Road Right Param
Mf_max = 80 # maximum of front road right (m)
Mf_min = 30 # minimum of front road right (m)
Mf_delta = Mf_max - Mf_min
Mb = 10 # behind road right (m)
Mf = Mb + vel_max*vel_max/(2*acc_max)
Mw = 10 # width of road right from reference line (m)


# Map Param
# ------------------------------------
# Reference generation
# Reference[dir][point]
# dir:
#                      /      /
#                   (4/5)   0/3
#                    /      /
#     —— 2/5 ——
#
#     ——(0/1)——
#                    \       \
#                    1\4    (2\3)
#                      \       \
# dir:0(left)/1(right)
# point:(x,y)
# ------------------------------------
Bound = 1000 # Control Zone boundary(m)
Rw = 60 # Road width
Ref_l = 60 # Fork distance from center in s_ais for trunk1(m)
Ref_w = 15 # Fork distance from center in d_ais for trunk1(m)
Reference = np.load(path_, allow_pickle=True)
# Theta_r_list[dir][point]
# point:(T,N)
Theta_r_list = [[] for i in range(6)]
RoadLine = [[] for i in range(3)]
RoadRight = [[] for i in range(12)]
# Points for generate Reference line
InitPoint = [
    [
        (-2*Ref_l, -Ref_w),
        (-Ref_l, -Ref_w),
        ((-Ref_l + Ref_l / 2 + math.sqrt(3) * Ref_w / 2) / 2, (-Ref_w + math.sqrt(3) * Ref_l / 2 - Ref_w / 2) / 2),
        (Ref_l / 2 + math.sqrt(3) * Ref_w / 2, math.sqrt(3) * Ref_l / 2 - Ref_w / 2),
        (Ref_l + math.sqrt(3) * Ref_w / 2, math.sqrt(3) * Ref_l - Ref_w / 2),
    ],
    [
        (-2*Ref_l, -Ref_w),
        (-Ref_l, -Ref_w),
        ((-Ref_l + Ref_l/2 - math.sqrt(3)*Ref_w/2)/2, (-Ref_w - math.sqrt(3)*Ref_l/2 - Ref_w/2)/2),
        (Ref_l/2 - math.sqrt(3)*Ref_w/2, -math.sqrt(3)*Ref_l/2 - Ref_w/2),
        (Ref_l - math.sqrt(3)*Ref_w/2, -math.sqrt(3)*Ref_l - Ref_w/2),
    ],
]


colors = [[99,178,238],
[118,218,145],
[248,203,127],
[248,149,136],
[124,214,207],
[145,146,171]]

def mymodf(x, y):
    return x - y*math.floor(x/y)


def mod2pi(theta):
    return mymodf(theta, 2*math.pi)


def MirrorFlip():
    for num in range(2, 6):
        j = num % 2
        mul = int(num/2)
        for p in Reference[j]:
            l_temp = math.sqrt(p[0]*p[0] + p[1]*p[1])

            r_temp = mod2pi(math.atan2(p[1], p[0]) + mul*math.radians(120))

            x = l_temp*math.cos(r_temp)
            y = l_temp*math.sin(r_temp)

            Reference[num].append([x,y])


def B(n,i,t):
    return math.factorial(n)/(math.factorial(i)*math.factorial(n-i)) * t**i * (1-t)**(n-i)


def BezierSmooth(p):
    res = []

    n = len(p) - 1
    c = 1/n
    for t in np.arange(0, 1+c, c):
        x0, y0 = 0, 0
        for i in range(n+1):
            x0 += B(n, i, t)*p[i][0]
            y0 += B(n, i, t)*p[i][1]

        res.append([x0, y0])

    return res


def BezierSmooth_1(p):
    res = []

    n = len(p) - 1
    c = 1/n
    for t in np.arange(0, 1+c, c):
        x0 = 0, 0
        for i in range(n + 1):
            x0 += B(n, i, t)*p[i]

        res.append(x0)

    return res


def Bezier(p, n, c, r, pf, j):
    temp = math.sqrt((p[0][0]-pf[0])*(p[0][0]-pf[0])+(p[0][1]-pf[1])*(p[0][1]-pf[1]))
    x1, y1 = p[0][0], p[0][1]

    delta_distance = 0

    for t in np.arange(0, 1+c, c):
        x0, y0 = 0, 0
        for i in range(n+1):
            x0 += B(n,i,t) * p[i][0]
            y0 += B(n,i,t) * p[i][1]

        dx = x0 - x1
        dy = y0 - y1
        x1 = x0
        y1 = y0
        temp += math.sqrt(dx*dx + dy*dy)

        if temp >= r:
            # theta = math.atan2(dy, dx)
            # res.append((x0,y0,theta))
            Reference[j].append([x0, y0])
            temp -= r
            dx1 = x0-p[-1][0]
            dy1 = y0-p[-1][1]
            delta_distance = math.sqrt((dx1*dx1) + (dy1*dy1))
            if delta_distance < r:
                break

    return delta_distance


def GenerateReferencePoint():
    for x in np.arange(-Bound, -2*Ref_l+s_step, s_step):
        Reference[0].append([x, -Ref_w])
        Reference[1].append([x, -Ref_w])

    temp_l = len(Reference[0])
    delta_distance = Bezier(InitPoint[0], len(InitPoint[0])-1, 0.0001, s_step, Reference[0][-1], 0)
    temp_d = s_step - delta_distance
    Reference[0].append([InitPoint[0][-1][0] + temp_d/2, InitPoint[0][-1][1] - math.sqrt(3)*temp_d/2])
    for i in range(temp_l):
        x_temp, y_temp = Reference[0][-1]
        Reference[0].append([x_temp + s_step/2, y_temp + math.sqrt(3)*s_step/2])

    temp_l = len(Reference[1])
    delta_distance = Bezier(InitPoint[1], len(InitPoint[1])-1, 0.0001, s_step, Reference[1][-1], 1)
    temp_d = s_step - delta_distance
    Reference[1].append([InitPoint[1][-1][0] + temp_d/2, InitPoint[1][-1][1] - math.sqrt(3)*temp_d/2])
    for i in range(temp_l):
        x_temp, y_temp = Reference[1][-1]
        Reference[1].append([x_temp + s_step/2, y_temp - math.sqrt(3)*s_step/2])

    MirrorFlip()

    for i in range(len(Reference)):
        Reference[i][800:-800] = BezierSmooth(Reference[i][800:-800])

    file = open('RefPath', 'w')
    file.truncate(0)
    file.close()

    refline = np.array(Reference)
    np.save('RefPath', refline)


def CalculateThetaR():
    for i in range(len(Reference)):
        lenth = len(Reference[i])
        for j in range(lenth):
            if j == 0:
                dx = Reference[i][j+1][0] - Reference[i][j][0]
                dy = Reference[i][j+1][1] - Reference[i][j][1]
                Theta_r_list[i].append(mod2pi(math.atan2(dy, dx)))
            elif j == lenth-1:
                dx = Reference[i][j][0] - Reference[i][j-1][0]
                dy = Reference[i][j][1] - Reference[i][j-1][1]
                Theta_r_list[i].append(mod2pi(math.atan2(dy, dx)))
            else:
                dx = Reference[i][j+1][0] - Reference[i][j-1][0]
                dy = Reference[i][j+1][1] - Reference[i][j-1][1]
                Theta_r_list[i].append(mod2pi(math.atan2(dy, dx)))


def GenerateRoadLine():
    for i in range(3):
        j = 1 + 2*i

        for index in range(len(Reference[j])):
            rs = index
            rx = Reference[j][index][0]
            ry = Reference[j][index][1]
            rtheta = Theta_r_list[j][index]
            s_condition = rs
            d_condition = -Rw
            x, y = frenet_to_cartesian1D(rs, rx, ry, rtheta, s_condition, d_condition)
            RoadLine[i].append([x, y])


def GenerateRoadRight():
    for i in range(6):
        for index in range(len(Reference[i])):
            rs = index
            rx = Reference[i][index][0]
            ry = Reference[i][index][1]
            rtheta = Theta_r_list[i][index]
            s_condition = rs
            d_condition = -Mw
            x, y = frenet_to_cartesian1D(rs, rx, ry, rtheta, s_condition, d_condition)
            RoadRight[int(i*2)].append([x, y])

            d_condition = Mw
            x, y = frenet_to_cartesian1D(rs, rx, ry, rtheta, s_condition, d_condition)
            RoadRight[int(i*2+1)].append([x, y])


def init_RefLine():
    CalculateThetaR()
    GenerateRoadLine()
    GenerateRoadRight()
