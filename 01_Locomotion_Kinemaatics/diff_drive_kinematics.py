#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
from math import cos, sin

fig, ax = plt.subplots()
dot, = ax.plot([], [], 'ro')
x = list()
y = list()


def update_dot(newd):
    dot.set_data(newd[0], newd[1])
    return dot,


def gen_dot():
    for data in zip(x, y):
        newdot = [data[0], data[1]]
        yield newdot


def init():
    ax.set_xlim(-5, 5)
    ax.set_ylim(0, 10)
    return plt.plot(x, y)


def main():
    '''
    l 1m
    v_l 4m/s
    v_r 6m/s
    '''
    l = 1
    v_l = 4
    v_r = 6
    # R = l * (v_l + v_r) / 2 / (v_r - v_l)
    # w = (v_r - v_l) / l
    theta = list()
    x_t = 0
    y_t = 0
    theta_t = 0

    delta_t = 0.01
    for _ in range(0, int(100 / delta_t)):
        x_t += 0.5 * (v_l + v_r) * cos(theta_t) * delta_t
        y_t += 0.5 * (v_l + v_r) * sin(theta_t) * delta_t
        theta_t += (v_r - v_l) / l * delta_t
        x.append(x_t)
        y.append(y_t)
        theta.append(theta_t)

    ani = animation.FuncAnimation(
        fig, update_dot, frames=gen_dot, interval=10, init_func=init)
    ani.save('diff_{0}_{1}.gif'.format(v_l, v_r), writer='imagemagick', fps=60)
    plt.show()


if __name__ == '__main__':
    main()
