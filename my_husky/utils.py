import numpy as np
from typing import Optional, List, Union

np.set_printoptions(precision=3, suppress=False, linewidth=200)


def smooth_velocity(init_pos, target_pos, max_v=0.5, smooth_frac=0.5, v=5/3 * 0.01):
    # v = 5/3 * 0.01
    diff = target_pos - init_pos
    steps = abs(diff) / (max_v * v)
    steps = np.math.ceil(max(steps, 1 / smooth_frac))
    max_v = diff / steps / v

    smooth_steps = np.math.ceil(steps * smooth_frac * 0.5)
    up = np.linspace(0, max_v, smooth_steps + 1)[1:]
    down = np.linspace(max_v, 0, smooth_steps + 1)
    rest = np.ones(steps - smooth_steps - 1) * max_v
    return np.concatenate((up, rest, down))


def velocity_control(init_pos: np.ndarray,
                     target_pos: np.ndarray,
                     max_v: np.ndarray,
                     smooth_frac: np.ndarray = 0.5,
                     v=5/3 * 0.01):
    diff = target_pos - init_pos
    max_v = np.maximum(np.sign(diff) * max_v, 1e-5)
    steps = abs(diff) / (max_v * v)
    steps = np.where(steps < 1 / smooth_frac, 1 / smooth_frac, steps)
    # steps[diff == 0] = 1
    # steps = np.ceil(np.maximum(steps, 1 / smooth_frac)).astype(int)
    # steps[steps < 1 / smooth_frac] = np.ceil(1 / smooth_frac).astype(int)
    max_v = diff / steps / v

    steps = np.ceil(steps).astype(int)
    smooth_steps = np.ceil(steps * smooth_frac * 0.5).astype(int)
    velocities = np.zeros((max(steps + smooth_steps), len(steps)))
    for i in range(len(steps)):
        up = np.linspace(0, max_v[i], smooth_steps[i] + 1)[1:]
        down = np.linspace(max_v[i], 0, smooth_steps[i] + 1)
        rest = np.ones(steps[i] - smooth_steps[i] - 1) * max_v[i]

        lengths = np.cumsum([len(up), len(rest), len(down)])
        velocities[:lengths[0], i] = up
        velocities[lengths[0]:lengths[1], i] = rest
        velocities[lengths[1]:lengths[2], i] = down

    return velocities

# init_pos = np.random.randn(10) + 5
# init_pos = np.zeros(10)
# target_pos = np.arange(10)
# max_v = 2
# vels = velocity_control(init_pos, target_pos, max_v)
# print(np.sum(vels, axis=0) *5/3*0.01 + init_pos)
# print(1)
#
# init = np.array([1])
# target = np.array([13])
# max_v = np.array([0.01555])
# smooth_frac = np.array([0.5])
# # v = 5/3 * 0.01
# v = 1
# vel = velocity_control(init, target, max_v, smooth_frac, v=v)
# print(vel)

# import matplotlib.pyplot as plt
#
# init = np.zeros(5)
# target = np.arange(5) - 2
# max_v = (np.random.randint(1, 5, 5) * np.sign(target)) * 0.01
# smooth_frac = np.random.rand(5) + 0.05
#
# vel = velocity_control(init, target, max_v, smooth_frac)
# cumsum = np.cumsum(vel, axis=0) * 5/3 * 0.01 + init
#
# fig, ax = plt.subplots(2, 1, figsize=(10, 10))
# for i in range(5):
#     ax[0].plot(vel[:, i], label=f'axis {i}')
#     ax[1].plot(cumsum[:, i], label=f'axis {i}')
#
# ax[0].legend()
# ax[1].legend()
# plt.show()