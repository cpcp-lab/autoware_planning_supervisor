import numpy as np
import random
import yaml
import lanelet2
import lanelet2.geometry
#import tf_transformations
from autoware_lanelet2_extension_python.projection import MGRSProjector
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from autoware_planning_supervisor.test_lanelet2 import plot_llt

def quaternion_from_euler(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = {}
    q["x"] = sr * cp * cy - cr * sp * sy
    q["y"] = cr * sp * cy + sr * cp * sy
    q["z"] = cr * cp * sy - sr * sp * cy
    q["w"] = cr * cp * cy + sr * sp * sy

    return q

def test_projection(lat=0.0, lon=0.0):
    return MGRSProjector(lanelet2.io.Origin(lat, lon))


def test_io(map_path, projection):
    return lanelet2.io.load(map_path, projection)

def generate_planning_conf(filename, config_fn):
    # Load the config file.
    with open(config_fn, encoding='utf-8') as f:
        config = yaml.safe_load(f)
    lat = config["/**"]["ros__parameters"]["map_origin"]["latitude"]
    lon = config["/**"]["ros__parameters"]["map_origin"]["longitude"]

    proj = test_projection(lat, lon)
    lanelet_map = test_io(filename, proj)

    rules_map = {"vehicle": lanelet2.traffic_rules.Participants.Vehicle,
                 "bicycle": lanelet2.traffic_rules.Participants.Bicycle,
                 "pedestrian": lanelet2.traffic_rules.Participants.Pedestrian,
                 "train": lanelet2.traffic_rules.Participants.Train}

    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  rules_map["vehicle"])
    routing_cost = lanelet2.routing.RoutingCostDistance(0.)  # zero cost for lane changes

    graph = lanelet2.routing.RoutingGraph(lanelet_map, traffic_rules, [routing_cost])

    #

    lanelets = list(lanelet_map.laneletLayer)
    ll_pair = random.sample(lanelets, 2)

    conf = ({}, {})

    ## Between two randomly chosen lanelets.
    #for i, llt in enumerate(ll_pair):
    #    assert len(llt.centerline) >= 2, print('Too small number of points')

    #    m_ind = len(llt.centerline)-2
    #    pos = llt.centerline[m_ind]
    #    conf[i]["position"] = {}
    #    conf[i]["position"]["x"] = pos.x
    #    conf[i]["position"]["y"] = pos.y
    #    conf[i]["position"]["z"] = pos.z

    #    p_next = llt.centerline[m_ind+1]
    #    dx = p_next.x - pos.x 
    #    dy = p_next.y - pos.y
    #    yaw = np.arctan2(dy, dx)
    #    quat = quaternion_from_euler(0, 0, yaw)
    #    #quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
    #    conf[i]["orientation"] = quat
    #    #conf[i]["orientation"] = {}
    #    #conf[i]["orientation"]["x"] = quat[0]
    #    #conf[i]["orientation"]["y"] = quat[1]
    #    #conf[i]["orientation"]["z"] = quat[2]
    #    #conf[i]["orientation"]["w"] = quat[3]

    # From one end to the other of a lanelet.
    llt = ll_pair[0]
    for i in [0, 1]:
        assert len(llt.centerline) >= 2, print('Too small number of points')

        if i == 0:
            m_ind = 0
        else:
            m_ind = len(llt.centerline)-2
        pos = llt.centerline[m_ind]
        conf[i]["position"] = {}
        conf[i]["position"]["x"] = pos.x
        conf[i]["position"]["y"] = pos.y
        conf[i]["position"]["z"] = pos.z

        p_next = llt.centerline[m_ind+1]
        dx = p_next.x - pos.x 
        dy = p_next.y - pos.y
        yaw = np.arctan2(dy, dx)
        quat = quaternion_from_euler(0, 0, yaw)
        #quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        conf[i]["orientation"] = quat
        #conf[i]["orientation"] = {}
        #conf[i]["orientation"]["x"] = quat[0]
        #conf[i]["orientation"]["y"] = quat[1]
        #conf[i]["orientation"]["z"] = quat[2]
        #conf[i]["orientation"]["w"] = quat[3]

    return conf

# eof
