import vrep
import random
import math
import sys
import time

class drivingEnv(object):

    def __init__(self):
        vrep.simxFinish(-1)
        self.clientId = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
        if self.clientId != -1:
            print('Connected to remote API server')
        else:
            print('Connection to V-Rep server failed')
            sys.exit('Could Not Connect')

        vrep.simxSynchronous(self.clientId, True)

        self.action_set= [0, 1, 2] # forward, left, right
        self.step_forward = (1.6+1.6)/10
        self.step_side = 0.6
        self.random_box=[5,1]
        self.get_handles()

        self.reset()


    def get_handles(self):
        _, self.car = vrep.simxGetObjectHandle(self.clientId, "car", vrep.simx_opmode_blocking)
        _, self.obstruction = vrep.simxGetObjectHandle(self.clientId, "obstruction", vrep.simx_opmode_blocking)
        _, self.dist_sensor = vrep.simxGetObjectHandle(self.clientId, "Proximity_sensor", vrep.simx_opmode_blocking)


    def destroy(self):
        vrep.simxStopSimulation(self.clientId, vrep.simx_opmode_oneshot)

    def reset(self):
        #print("sim reset")
        vrep.simxStopSimulation(self.clientId, vrep.simx_opmode_blocking)
        self.randomize_setup()

        #time.sleep(0.5)
        vrep.simxStartSimulation(self.clientId, vrep.simx_opmode_blocking)
        return [1, 0, 0, 3]

    def randomize_block(self):
        random_x = random.randint(4,8)
        random_y = random.randint(1,2)
        random_yb = random.randint(1,2)
        if random_y == 2:
            random_yb = 2
        self.random_box = [random_x, random_y]
        new_obstruction_pos = [1.6-(self.step_forward*random_x),-0.6+(self.step_side*random_y),0.15]
        new_car_pos = [1.6-(self.step_forward*0),-0.6+(self.step_side*random_yb),0.15]
        vrep.simxSynchronousTrigger(self.clientId)
        vrep.simxSetObjectPosition(self.clientId, self.obstruction, -1, new_obstruction_pos, vrep.simx_opmode_oneshot)
        vrep.simxSynchronousTrigger(self.clientId)
        vrep.simxSetObjectPosition(self.clientId, self.car, -1, new_car_pos, vrep.simx_opmode_oneshot)

    def step(self, action):
        _, current_car_position = vrep.simxGetObjectPosition(self.clientId, self.car, -1, vrep.simx_opmode_oneshot_wait)
        if action == 0:
            current_car_position[0] = current_car_position[0] - self.step_forward
        elif action == 1:
            current_car_position[1] = current_car_position[1] - self.step_side
        elif action == 2:
            current_car_position[1] = current_car_position[1] + self.step_side
        else:
            print("ENV - Invalid Action")
        vrep.simxSynchronousTrigger(self.clientId)
        vrep.simxSetObjectPosition(self.clientId, self.car, -1, current_car_position, vrep.simx_opmode_oneshot_wait)

        observations = []
        #time.sleep(0.05)
        block_car_position = ["x","y"]
        _, current_car_position = vrep.simxGetObjectPosition(self.clientId, self.car, -1, vrep.simx_opmode_oneshot_wait)
        block_car_position[0] = int(round((1.6 - current_car_position[0])/self.step_forward))
        block_car_position[1] = int(round((current_car_position[1] + 0.6 )/self.step_side))     
        #print(block_car_position)   
        observations.extend(block_car_position)

        observations.append(action)

        vrep.simxReadProximitySensor(self.clientId, self.dist_sensor, vrep.simx_opmode_remove)
        _, det_state, det_point, det_obj_handle, det_surf_norm_vec = vrep.simxReadProximitySensor(self.clientId, self.dist_sensor, vrep.simx_opmode_blocking)
        if det_state:
            observations.append(int(self.get_euc_dist(det_point)))
        else:
            observations.append(2)
        print(observations)

        if current_car_position[0]<-1.8 or current_car_position[0]>1.8 or current_car_position[1]<-0.9 or current_car_position[1]>0.9 or (block_car_position[0]==self.random_box[0] and block_car_position[1]==self.random_box[1]):
            #print(current_car_position)
            a= self.reset()
            return a, True

            #return observations, True
        else:
            return observations, False
        
        # return observations

    def step2(self, state, action):
        new_state=list(state)
        if(action==0):
            new_state[0]=state[0]+1
        elif(action==1):
            new_state[1]=state[1]-1
        elif(action==2):
            new_state[1]=state[1]+1
        else:
            return [0,0,action,3], True

        #setting sensor value
        if((new_state[0]==self.random_box[0]-4 or new_state[0]==self.random_box[0]-3) and (new_state[1]==self.random_box[1])):
            new_state[3]=1
        elif((new_state[0]==self.random_box[0]-2 or new_state[0]==self.random_box[0]-1) and (new_state[1]==self.random_box[1])):
            new_state[3]=0
        else:
            new_state[3]=2
        new_state[2]=action
        # if((new_state[0]==1 or new_state[0]==2) and (new_state[1]==1)):
        #     new_state[3]=1
        # elif((new_state[0]==3 or new_state[0]==4) and (new_state[1]==1)):
        #     new_state[3]=0
        # else:
        #     new_state[3]=2
        # new_state[2]=action

        #checking exist state
        if(new_state[0]>10 and new_state[1]==1):
            return [1,0,action,3], True

        if(new_state[0]<0 or new_state[0]>10 or new_state[1]<0 or new_state[1]>2 or (new_state[0]==self.random_box[0] and new_state[1] == self.random_box[1])):
            #time.sleep(1)
            #print(state, action, new_state)
            #print("---------------------")
            return [0,0,action,3], True
        #time.sleep(1)
        #print(state, action, new_state)
        #print("---------------------")
        return new_state, False

    def randomize_setup(self):
        car_x_range = [1.6, -1.6]
        car_y_range = [-0.8, 0.8]
        x_start = 1.6
        y_start = -0.6
        y_pos = random.randint(0, 2)
        new_car_pos = [1.6-(self.step_forward*0),-0.6+(self.step_side*1),0.052]
        x_pos = random.randint(1, 10)
        y_pos = random.randint(0, 2)

        new_obstruction_pos = [1.6-(self.step_forward*5),-0.6+(self.step_side*1),0.15]
        vrep.simxSynchronousTrigger(self.clientId)
        vrep.simxSetObjectPosition(self.clientId, self.car, -1, new_car_pos, vrep.simx_opmode_oneshot)
        vrep.simxSynchronousTrigger(self.clientId)
        vrep.simxSetObjectPosition(self.clientId, self.obstruction, -1, new_obstruction_pos, vrep.simx_opmode_oneshot)

        _, current_car_position = vrep.simxGetObjectPosition(self.clientId, self.car, -1, vrep.simx_opmode_buffer)


    def get_euc_dist(self, xyz_coord):
        dist = 0
        for loc in xyz_coord:
            dist = dist + loc**2
        return dist ** 0.5
