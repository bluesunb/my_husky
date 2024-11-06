from hf_env import HuskyEnv
from stable_baselines3 import PPO
from IPython.display import clear_output

import numpy as np
import time

np.set_printoptions(precision=3, suppress=True)

policy_path = "cnn_policy/hf_policy_new3.zip"

my_env = HuskyEnv(headless=False)
model = PPO.load(policy_path)

for i in range(20):
    obs = my_env.reset()
    done = False
    # my_env.goal.set_world_pose(np.array([ 0.866, -0.06 ,  0.345]))
    j = 0
    while not done:
        actions, _ = model.predict(observation=obs, deterministic=True)
        clear_output(wait=True)
        obs, reward, done, info = my_env.step(actions, debug=False)
        # if actions[-2] == 1:
        # print(f'!!! : {j}, action: {actions}')
        my_env.render()
        j += 1

    clear_output(wait=True)
    print(f'DONE: {j}, {info}')
    print(f'===={my_env.husky.get_joint_positions()}')
    if info['msg'] == "reach goal":
        print('reach goal')
        time.sleep(1)

my_env.close()

