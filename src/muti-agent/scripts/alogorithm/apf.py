import sys
import math
import numpy as np
import matplotlib.pyplot as plt


def calcDistance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def calcCost(x, y, x_l, y_l, x_o, y_o):
    a_o = 1
    u_o = 4
    a_l = 1
    u_l = 4

    cost_o = 0
    for i in range(len(x_o)):
        d_o = calcDistance(x, y, x_o[i], y_o[i])
        cost_o += a_o * math.exp(-u_o * (d_o ** 2))
        pass

    # d_l = calcDistance(x, y, x_l, y_l)
    # cost_l = -a_l * math.exp(-u_l * (d_l ** 2))
    cost_l = 0
    cost = cost_o + cost_l
    return cost


def genPoints(N, x, y):
    """
    @return: 2xN np.array
    """
    step = 0.2
    theta = np.linspace(0, 2 * np.pi, N)
    points = np.zeros((2, N))
    for i in range(N):
        points[0, i] = x + step * np.cos(theta[i])
        points[1, i] = y + step * np.sin(theta[i])

    return points


def chooseBestPoint(N, cost_err, d_l_err, fitness, bacterias):
    """
    @return: 2x1 np.array
    """
    best_point = np.zeros((2, 1))
    for i in range(N):
        k = np.argmax(fitness)
        if cost_err[0, k] < 0.35:
            if d_l_err[0, k] < 0:
                best_point[0, 0] = bacterias[0, k]
                best_point[1, 0] = bacterias[1, k]
                break
            else:
                fitness[0, k] = 0
                pass
            pass
        else:
            fitness[0, k] = 0
            pass
        pass
    return best_point


def apf(x, y, x_l, y_l, x_o, y_o, N):
    current_cost = calcCost(x, y, x_l, y_l, x_o, y_o)
    d_l = calcDistance(x, y, x_l, y_l)
    bacterias = genPoints(N, x, y)
    bacterias_costs = np.zeros((1, N))
    bacterias_d_ls = np.zeros((1, N))
    for i in range(N):
        bacterias_costs[0, i] = calcCost(
            bacterias[0, i], bacterias[1, i], x_l, y_l, x_o, y_o
        )
        bacterias_d_ls[0, i] = calcDistance(bacterias[0, i], bacterias[1, i], x_l, y_l)
        pass

    cost_err = np.zeros((1, N))
    d_l_err = np.zeros((1, N))
    fitness = np.zeros((1, N))
    for i in range(N):
        # cost_err[0, i] = bacterias_costs[0, i] - current_cost
        cost_err[0, i] = bacterias_costs[0, i]
        d_l_err[0, i] = bacterias_d_ls[0, i] - d_l
        fitness[0, i] = -d_l_err[0, i]
        pass

    best_bacteria = chooseBestPoint(N, cost_err, d_l_err, fitness, bacterias)

    if d_l > 0.2:
        x_d, y_d = best_bacteria[0, 0], best_bacteria[1, 0]
    else:
        x_d, y_d = x_l, y_l
    return x_d, y_d


def main(args):
    x, y = 0, 0  # self
    x_l, y_l = 9, 9  # leader
    # x_o, y_o = 1, 1  # another follower
    x_o = [1, 4]
    y_o = [1, 3]
    N = 60
    plt.figure()
    plt.grid()

    plt.scatter(x, y, c="r")
    plt.scatter(x_l, y_l, c="b")
    plt.scatter(x_o, y_o, c="g")

    for i in range(100):

        x_d, y_d = apf(x, y, x_l, y_l, x_o, y_o, N)
        if i == 41:
            print()
        # if x_d == 0:
        #     print(i)
        print(x_d, y_d)
        plt.scatter(x_d, y_d, c="y")
        x, y = x_d, y_d
        pass
    plt.show()

    pass


if __name__ == "__main__":
    main(sys.argv)
    pass
