import numpy as np


x_d = 1
y_d = 1
x = 1
y = 0
theta = np.pi / 2
l = 0.2
k = 0.2


def gzy2_stabilize():
    e_x = x_d - x
    e_y = y_d - y
    u_x = k * e_x
    u_y = k * e_y
    A = np.array(
        [
            [np.cos(theta), -l * np.sin(theta)],
            [np.sin(theta), l * np.cos(theta)],
        ]
    )
    A_i = np.linalg.inv(A)
    U = np.array([[u_x], [u_y]])
    v_w = np.linalg.solve(A, U)
    v_w_2 = np.matmul(A_i, U)
    v = v_w[0]
    w = v_w[1]
    return v, w


v, w = gzy2_stabilize()
print(v, w)
