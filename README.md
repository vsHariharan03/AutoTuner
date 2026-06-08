# Nav2 AutoTuner


## Overview

We automate the tuning of the Nav2 MPPI (Model Predictive Path Integral) controller parameters using Bayesian Optimization.

And allows users to run multiple instances of gazebo in parallel to speed up the Optimization process.

The system evaluates navigation performance in simulation, computes a performance score, and iteratively searches for better controller parameters using Bayesian Optimization.

---

The architecture consists of four major components:

- Bayesian Optimization Server
- Socket Worker Layer
- Score Accumulator Node
- Navigation Evaluation and Reset System

---

## System Architecture 

The system has four pieces that work together:
- Bayesian Optimization Server
The brain of the operation. It keeps track of all previous simulation results and uses them to decide which parameters to try next. It talks to the rest of the system over a Unix socket at /tmp/my_socket.

- Socket Workers
Think of these as middlemen — one per simulation instance. Each worker has its own socket (/tmp/my_socket0, /tmp/my_socket1, etc.) and handles passing scores up to the optimizer and new parameters back down. By default, up to 8 simulations can run in parallel, though you can increase this.

- Score Accumulator (ROS Node)
Runs inside each simulation. It collects navigation scores from ROS topics and forwards them to its assigned worker socket. It also receives new parameters back and applies them to the running controller. Each simulation is isolated using a different ROS_DOMAIN_ID, which maps directly to its socket — so domain 3 talks to /tmp/my_socket3.

- Navigation Evaluator & Reset System
The automated driver. It sends the robot through a predefined set of waypoints (loaded from goals.npy), watches how it does, and resets the simulation when it's time for the next trial.


---
## Getting Started

**Clone the repository**
```bash
git clone https://github.com/vsHariharan03/AutoTuner.git
```

**Navigate into the project**
```bash
cd nav2_autotuner
```

**Build the workspace**
```bash
colcon build --symlink-install
```

**Running**

Running the Server to manage all the parallel simulations and communicates between the optimizer and the simulation via sockets

*right now the server_node can take upto 8 simulations but can be increases by changing the NUM_WORKERS parameter*

```bash
python3 ./src/accumulator_pkg/accumulator_pkg/server_node.py
```

Running the optimizer that tries out different parameters.
Communicates using the  /tmp/my_socket socket.

```bash
python3 ./src/accumulator_pkg/accumulator_pkg/optimizer.py
```

to run multiple simulations first you must :

- have a different ROS_DOMAIN_ID to ensure topics dont go mix up across different instances

- have different GZ_PORT to launch different Gazebo instances

Now launch nav2
```bash
ros2 launch navigation_pkg navigation.launch.py
```

then run :


```bash
ros2 run goal_gen accumulator 
```
which launches the accumulator which contacts the server via Sockets and sends scores and recieves new parameters.

the sockets are named as /tmp/my_socket<domain>

```bash
ros2 run goal_gen nav2_switcher
```
which is responsible for handling and setting the new parameters and reseting the simulation and sending the score to the accumulator

```bash
ros2 run goal_gen hard_coded
```

I run these hard_coded parameters to complete a course this node is responsible for actally calculating the score, deciding when the simulation should be reset by sending the score to the nav2_switcher service


Users can create a custom goal generation and checking algorithm as per you need.

To change trigger a change in the parameters you logic must send a service call on 'update_mppi_params'

---
