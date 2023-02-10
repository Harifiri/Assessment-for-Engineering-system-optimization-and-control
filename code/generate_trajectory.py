import os
from param import *
from coord_transform import frenet_to_cartesian1D

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
    # GenerateReferencePoint()
    # print("-----------------111------------------")
    path_ = os.path.join(os.path.dirname(os.getcwd()), 'RefPath.npy')
    global Reference
    Reference = np.load(path_, allow_pickle=True)

    CalculateThetaR()
    GenerateRoadLine()
    GenerateRoadRight()


