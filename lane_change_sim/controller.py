import env
from curtsies import FullscreenWindow, Input, FSArray
import numpy as np

e = env.drivingEnv()

with Input() as input_generator:
    trajectory = [[0, 1, 5, 2]]
    trajectories = []
    steps = 1

    for c in input_generator:
        if c == '<ESC>':
            break
        else:
            s = repr(c).replace("'","").replace("<","").replace(">","").lower()
            if s == "up":
            	s = 0
            elif s == "left":
            	s = 1
            elif s == "right":
            	s = 2
            else:
            	print("Invalid Action")

            if steps < 110:
                print(s)
                obs, _ = e.step(s)
                trajectory.append(obs)
                steps += 1

            else:
                trajectories.append(trajectory)
                trajectory = [[0, 1, 5, 2]]
                steps = 1

                np_trajectories = np.array(trajectories, float)
                print("Traj. Shape: ", np_trajectories.shape)
                np.save("expert_trajectories", arr=np_trajectories)
                print(np_trajectories)
                e.reset()
            
