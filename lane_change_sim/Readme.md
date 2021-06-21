# Learning to Drive Simulator


### Installation


Recommended [Ubuntu 16](http://releases.ubuntu.com/16.04/) to run the code


##### Installing Dependencies


  - Install latest version of [V-Rep Pro Edu](http://www.coppeliarobotics.com/downloads.html)
  - [Python 2.7](https://www.python.org/downloads/release/python-2715/) is required
  - Install latest version of tensorflow using [pip install tensorflow](https://www.tensorflow.org/install)


##### To run simulator


Navigate to where simulator is downloaded and use path of provided environment file


./vrep.sh driving_env.ttt


To run in headless mode
./vrep.sh -h driving_env.ttt


env.py - sets up the variables and various methods needed to interact with the simulator.

controller.py - calls env.py to enable control of the agent through keyboard and stores the performed trajectories in a different file.
