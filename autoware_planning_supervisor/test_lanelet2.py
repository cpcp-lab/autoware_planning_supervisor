from autoware_lanelet2_extension_python.projection import MGRSProjector
import autoware_lanelet2_extension_python.utility.query as query
import autoware_lanelet2_extension_python.utility.utilities as utilities
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import lanelet2
import lanelet2.geometry
import matplotlib.pyplot as plt
import numpy as np
import tf_transformations
import random
import argparse
import yaml

def test_projection(lat=0.0, lon=0.0):
    return MGRSProjector(lanelet2.io.Origin(lat, lon))


def test_io(map_path, projection):
    return lanelet2.io.load(map_path, projection)


def plot_ll2_id(ll2, ax, text):
    xs, ys = np.array([pt.x for pt in ll2.centerline]), np.array([pt.y for pt in ll2.centerline])
    x, y = np.average(xs), np.average(ys)
    ax.text(x, y, text)


def plot_linestring(linestring, ax, color, linestyle, label, **kwargs):
    xs = [pt.x for pt in linestring]
    ys = [pt.y for pt in linestring]
    ax.plot(xs, ys, color=color, linestyle=linestyle, label=label, **kwargs)

def plot_llt(llt):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.axis("equal")
    ax.set_title(f"Lanelet {llt.id}")

    # Print the number of points of the left/right bound.
    print(f"  Left Bound has {len(llt.leftBound)} points")
    print(f"  Right Bound has {len(llt.rightBound)} points")

    # Print the attributes.
    for key, value in llt.attributes.items():
        print(f"  Attribute: {key} = {value}")

    print("-" * 40)

    plot_linestring(llt.leftBound, ax, "blue", "--", "%d left" % llt.id)
    plot_linestring(llt.rightBound, ax, "red", "--", "%d right" % llt.id)
    plot_linestring(llt.centerline, ax, "orange", "--", "%d center" % llt.id)

    for pos in llt.centerline:
        ax.plot(pos.x, pos.y, 'x')

    plt.legend()
    plt.show()

def plot_ll_pair(ll_pair):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.axis("equal")
    ax.set_title("test")

    for _, llt in enumerate(ll_pair):
        print(f"Lanelet ID: {llt.id}")

        # Print the number of points of the left/right bound.
        print(f"  Left Bound has {len(llt.leftBound)} points")
        print(f"  Right Bound has {len(llt.rightBound)} points")

        # Print the attributes.
        for key, value in llt.attributes.items():
            print(f"  Attribute: {key} = {value}")

        print("-" * 40)

        plot_linestring(llt.leftBound, ax, "blue", "--", "%d left" % llt.id)
        plot_linestring(llt.rightBound, ax, "red", "--", "%d right" % llt.id)
        plot_linestring(llt.centerline, ax, "orange", "--", "%d center" % llt.id)

        m_ind = len(llt.centerline) // 2
        pos = llt.centerline[m_ind]
        #ax.scatter([pos.x], [pos.y], marker="o", label="centerline0")

        plt.annotate(f"{llt.id}", xy=(pos.x, pos.y), xytext=(pos.x+20, pos.y))

        #print(f"{pos.x}, {pos.y}, {pos.z}")

        if len(llt.centerline) < 2:
            print("Too small number of points")
            continue

        for i in [0, len(llt.centerline)-2]:
            pos = llt.centerline[i]
            print(f"  position:\n    x: {pos.x}\n    y: {pos.y}\n    z: {pos.z}")
            p_next = llt.centerline[i+1]

            plot_linestring([pos, p_next], ax, "black", "-", "pos %d" % i)

            dx = p_next.x - pos.x 
            dy = p_next.y - pos.y
            yaw = np.arctan2(dy, dx)
            quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
            print(f"  orientation:\n    x: {quat[0]}\n    y: {quat[1]}\n    z: {quat[2]}\n    w: {quat[3]}")

    llt = ll_pair[0]

    print(f"Lanelet ID: {llt.id}")

    # Print the number of points of the left/right bound.
    print(f"  Left Bound has {len(llt.leftBound)} points")
    print(f"  Right Bound has {len(llt.rightBound)} points")

    # Print the attributes.
    for key, value in llt.attributes.items():
        print(f"  Attribute: {key} = {value}")

    print("-" * 40)

    plot_linestring(llt.leftBound, ax, "blue", "--", "%d left" % llt.id)
    plot_linestring(llt.rightBound, ax, "red", "--", "%d right" % llt.id)
    plot_linestring(llt.centerline, ax, "orange", "--", "%d center" % llt.id)

    #plt.annotate(f"{llt.id}", xy=(pos.x, pos.y), xytext=(pos.x+20, pos.y))

    if len(llt.centerline) < 2:
        print("Too small number of points")
        exit()

    for i in [0, 1]:
        if i == 0:
            m_ind = 0
        else:
            m_ind = len(llt.centerline)-2
        pos = llt.centerline[m_ind]
        #ax.scatter([pos.x], [pos.y], marker="o", label="centerline0")

        #print(f"{pos.x}, {pos.y}, {pos.z}")

        pos = llt.centerline[m_ind]
        print(f"  position:\n    x: {pos.x}\n    y: {pos.y}\n    z: {pos.z}")
        p_next = llt.centerline[m_ind+1]

        plot_linestring([pos, p_next], ax, "black", "-", "pos %d" % i)
        ax.plot(pos.x, pos.y, 'x')
        ax.plot(p_next.x, p_next.y, 'o')

        dx = p_next.x - pos.x 
        dy = p_next.y - pos.y
        yaw = np.arctan2(dy, dx)
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        print(f"  orientation:\n    x: {quat[0]}\n    y: {quat[1]}\n    z: {quat[2]}\n    w: {quat[3]}")

    plt.legend()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="Path to the map osm file")
    parser.add_argument("config_fn", help="Path to the config yaml file")
    args = parser.parse_args()

    # Load the config file.
    with open(args.config_fn, encoding='utf-8') as f:
        config = yaml.safe_load(f)
    print(config)
    ## Map origin
    #lat = 35.23808753540768
    #lon = 139.9009591876285
    lat = config["/**"]["ros__parameters"]["map_origin"]["latitude"]
    lon = config["/**"]["ros__parameters"]["map_origin"]["longitude"]

    proj = test_projection(lat, lon)
    lanelet_map = test_io(args.filename, proj)

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

    # Plotting
    #plot_llt(ll_pair[0])
    plot_ll_pair(ll_pair)

# eof
