import rosbag
import sys
from kitti2bag import run_kitti2bag

data_path = "/home/ANT.AMAZON.COM/sappel/Downloads/training"

for index in range(1):
    scenario = str(index).zfill(4)
    rosbag = run_kitti2bag(scenario, data_path)

    # rosbag_name = scenario + ".bag"
    # with rosbag.Bag(rosbag_name, 'w') as outbag:
    #     for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
    #         if topic == "/kitti/velo/pointcloud":
    #             buffer = t
    #             outbag.write(topic,msg, msg.header.stamp)

    #         elif topic == "/tf" and msg.transforms:
    #             for i in range(len(msg.transforms)):
    #                 msg.transforms[i].header.stamp = buffer
    #                 outbag.write(topic, msg, msg.transforms[i].header.stamp)

    #         elif topic == "/tf_static" and msg.transforms:
    #             for i in range(len(msg.transforms)):
    #                 msg.transforms[i].header.stamp = buffer
    #                 outbag.write(topic, msg, msg.transforms[i].header.stamp)
    #         else:
    #             msg.header.stamp = buffer
    #             outbag.write(topic, msg, msg.header.stamp)
