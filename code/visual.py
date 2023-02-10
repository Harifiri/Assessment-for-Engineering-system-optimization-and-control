import time
import pygame
from param import *

color = [
    [0, 0, 0],
    [255, 0, 0],
    [0, 255, 0],
    [255, 0, 0],
]


def cps(para):  # change point size
    x, y = para
    x = x + w/2
    y = -y + h/2
    return x, y


def cls(list):  # change list size
    list_t = np.transpose(list)
    list_t[0] = list_t[0] + w/2
    list_t[1] = -list_t[1] + h/2

    return np.transpose(list_t)


def DrawBackground(screen):
    screen.fill((255, 255, 255))

    # pygame.draw.circle(screen, color[0], center_pos, inner_r, width=1)
    pygame.draw.circle(screen, color[0], center_pos, outer_r, width=1)
    for line in RoadLine:
        pygame.draw.aalines(screen, [0, 0, 0], False, cls(np.array(line)))


def DrawPointMessage(screen, point, road_right):
    color_index = 0
    #
    # color_index += 1
    # if color_index >= len(color):
    #     color_index = 0
    #
    # draw car
    s_ = int(point.s[0]/s_step)
    theta = Theta_r_list[point.dir][s_]
    #
    # if theta > Theta_r_list[point.dir][s_-2] and theta < Theta_r_list[point.dir][s_+2]:
    #     pass
    # else:
    #     print(s_)
    #
    x, y = Reference[point.dir][s_]

    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    x0 = x - car_w/2*sin_theta + car_lf*cos_theta
    x1 = x + car_w/2*sin_theta + car_lf*cos_theta
    x2 = x + car_w/2*sin_theta - car_lf*cos_theta
    x3 = x - car_w/2*sin_theta - car_lf*cos_theta

    y0 = y + car_w/2*cos_theta + car_lf*sin_theta
    y1 = y - car_w/2*cos_theta + car_lf*sin_theta
    y2 = y - car_w/2*cos_theta - car_lf*sin_theta
    y3 = y + car_w/2*cos_theta - car_lf*sin_theta

    points = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]

    pygame.draw.polygon(screen, colors[point.dir], cls(np.array(points)), 0)

    ## draw road right
    sb_ = max(int(road_right[1][0]/s_step), 0)
    sf_ = int(road_right[1][1]/s_step)

    # print(sb_, sf_)

    line1 = RoadRight[point.dir*2][sb_:sf_]
    line2 = RoadRight[point.dir*2+1][sb_:sf_]
    line3 = [RoadRight[point.dir*2][sb_], RoadRight[point.dir*2+1][sb_]]
    line4 = [RoadRight[point.dir*2][sf_], RoadRight[point.dir*2+1][sf_]]

    pygame.draw.aalines(screen, colors[point.dir], False, cls(np.array(line1)))
    pygame.draw.aalines(screen, colors[point.dir], False, cls(np.array(line2)))
    pygame.draw.aalines(screen, colors[point.dir], False, cls(np.array(line3)))
    pygame.draw.aalines(screen, colors[point.dir], False, cls(np.array(line4)))


def DrawTrajectory(trajectory_list, road_right_list, t_terminal):
    pygame.init()
    screen = pygame.display.set_mode((w, h))
    DrawBackground(screen)
    time.sleep(10)

    # time_0_ = time.time()
    # time_1_ = time.time()
    for t_temp in np.arange(0, t_terminal, t_step):
        time_0_ = time.time()
        DrawBackground(screen)
        for i in range(len(trajectory_list)):
            trajectory = trajectory_list[i]
            road_right = road_right_list[i]
            if trajectory[0].t > t_temp or trajectory[-1].t < t_temp:
                continue
            index = int((t_temp - trajectory[0].t+0.001)/t_step)
            print(len(trajectory), index)
            DrawPointMessage(screen, trajectory[index], road_right[index])

        pygame.display.update()
        pygame.display.flip()

        for event in pygame.event.get():
            # check if the event is the X button
            if event.type == pygame.QUIT:
                # if it is quit the game
                pygame.quit()
                exit(0)

        print(round(t_temp, 1))
        time_1_ = time.time()
        time_sleep_ = max(0.0, 0.025 - (time_1_-time_0_))
        time.sleep(time_sleep_)

    time.sleep(1)
    screen.fill((255, 255, 255))
    pygame.display.update()
    pygame.display.flip()
    pygame.quit()
    exit(0)


# if __name__ == '__main__':
#     ST = STGraph(12, 2000 + car_lf + Mf_max)
#
#     # for i in range(len(Reference[0])):
#     #     p = Reference[0][i]
#     #     if p[0]*p[0]+p[1]*p[1] <= outer_r*outer_r:
#     #         print(i)
#     #         break
#     # while True:
#     #     pass
#
#     # v_temp = 7.5
#     # a_temp = 0
#     # s_temp = s_offset_1_ - (car_lf + Mf_min + Mf_delta*(v_temp/vel_max)*(v_temp/vel_max))
#     # init_p = Point(0, [s_temp, v_temp, a_temp], 0)
#     # planner_ = Planner(ST.graph, init_p)
#     # trajectory = planner_.SolveTrajectory()
#     # ST.update_path(trajectory)
#     # TrajectoryList.append(trajectory)
#     # t_max = max(t_max, init_p.t+len(trajectory)*t_step)
#
#     # graph = ST.graph[3]
#     # graph = np.transpose(graph)
#     # plt.imshow(graph, cmap='binary', origin='lower')
#     #
#     # plt.show()
#
#     # graph = ST.graph[2]
#     # graph = np.transpose(graph)
#     # plt.imshow(graph, cmap='binary', origin='lower')
#     #
#     # plt.show()
#
#     pygame.init()
#     screen = pygame.display.set_mode((w, h))
#     DrawBackground(screen)
#     time.sleep(2)
#     for t_temp in np.arange(0, t_max, t_step):
#         DrawBackground(screen)
#         for trajectory in TrajectoryList:
#             if trajectory[0].t > t_temp or trajectory[-1].t < t_temp:
#                 continue
#             index = int( (t_temp - trajectory[0].t)/t_step )
#             DrawPointMessage(screen, trajectory[index])
#
#         pygame.display.update()
#         pygame.display.flip()
#
#         time.sleep(0.01)
#
#     pygame.display.update()
#     pygame.display.flip()
#     for event in pygame.event.get():
#         # check if the event is the X button
#         if event.type == pygame.QUIT:
#             # if it is quit the game
#             pygame.quit()
#             exit(0)
#
#     time.sleep(10000)
#     screen.fill((255, 255, 255))
#     pygame.display.update()
#     pygame.display.flip()