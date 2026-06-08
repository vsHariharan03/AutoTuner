import socket
import os
import json

from bayes_opt import BayesianOptimization
from bayes_opt import acquisition



socket_path = "/tmp/my_socket"

if os.path.exists(socket_path):
    os.remove(socket_path)

server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
server.bind(socket_path)
server.listen()

print("Optimizer server listening")


acq = acquisition.UpperConfidenceBound(kappa=2.5)


bounds = pbounds = {
    # Discrete/Integer-like
    'FollowPath.time_steps': (25, 40, int),
    'FollowPath.batch_size': (1950, 2000, int),

    # Model Dynamics
    'FollowPath.model_dt': (0.05, 0.2),

    # Sampling Noise (Standard Deviations)
    'FollowPath.vx_std': (0.3, 1.0),
    'FollowPath.wz_std': (0.3, 1.0),

    # Velocity Limits
    'FollowPath.vx_max': (1.8, 2.23),
    'FollowPath.vx_min': (-0.5, 0.0),
    'FollowPath.wz_max': (2.0, 5.0),

    # Cost Function Parameters
    'FollowPath.temperature': (0.1, 1.0),
    'FollowPath.gamma': (0.001, 0.1),

    # Path Handling
    'FollowPath.prune_distance': (0.5, 3.0),

    # ConstraintCritic
    'FollowPath.ConstraintCritic.cost_weight': (1.0, 10.0),

    # GoalCritic
    'FollowPath.GoalCritic.cost_weight': (1.0, 10.0),
    'FollowPath.GoalCritic.threshold_to_consider': (0.5, 2.5),

    # GoalAngleCritic
    'FollowPath.GoalAngleCritic.cost_weight': (1.0, 10.0),
    'FollowPath.GoalAngleCritic.threshold_to_consider': (0.1, 1.0),

    # PreferForwardCritic
    'FollowPath.PreferForwardCritic.cost_weight': (1.0, 15.0),
    'FollowPath.PreferForwardCritic.threshold_to_consider': (0.1, 1.0),

    # ObstaclesCritic
    'FollowPath.ObstaclesCritic.repulsion_weight': (1.0, 15.0),
    'FollowPath.ObstaclesCritic.critical_weight': (10.0, 50.0),
    'FollowPath.ObstaclesCritic.collision_margin_distance': (0.1, 0.5),
    'FollowPath.ObstaclesCritic.near_goal_distance': (0.05, 0.5),

    # CostCritic
    'FollowPath.CostCritic.cost_weight': (1.0, 10.0),
    'FollowPath.CostCritic.critical_cost': (200.0, 400.0),
    'FollowPath.CostCritic.near_goal_distance': (0.5, 2.0),

    # PathAlignCritic
    'FollowPath.PathAlignCritic.cost_weight': (10.0, 50.0),
    'FollowPath.PathAlignCritic.max_path_occupancy_ratio': (0.01, 0.2),
    'FollowPath.PathAlignCritic.threshold_to_consider': (0.1, 1.0),
    'FollowPath.PathAlignCritic.offset_from_furthest': (5, 40, int)
}

optimizer = BayesianOptimization(
    f=None,
    acquisition_function=acq,
    pbounds=pbounds,
    verbose=2,
    random_state=1,
)


def optimizer_callback(data):
    response=""

    data=json.loads(data)

    if(float(data['score'])!=float(-1.0)):

        # print(type(data['params']))
        # print()
        # print(data['params'])
        # print()

        optimizer.register(
            params=data['params'],
            target=data['score'],
        )
    else:
        print("Starting for Domain : ",data['domain'])

    next_point_to_probe = optimizer.suggest()
    response=json.dumps(next_point_to_probe)

    optimizer.save_state("optimizer_state.json")

    # print("sent : ",response)

    return response

while True:

    conn, _ = server.accept()

    try:
        while True:

            data = conn.recv(4096)

            if not data:
                break

            msg = data.decode().strip()
            # print("Score received:", msg)

            response = optimizer_callback(msg)
            conn.sendall(response.encode())

    except Exception as e:
        print("Server error:", e)

    finally:
        conn.close()