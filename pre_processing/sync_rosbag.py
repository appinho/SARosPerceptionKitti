import rosbag
import sys

file = sys.argv[1]

with rosbag.Bag('synchronized_data.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
        if topic == "/kitti/velo/pointcloud":
            buffer = t
            outbag.write(topic,msg, msg.header.stamp)

        elif topic == "/tf" and msg.transforms:
            for i in range(len(msg.transforms)):
                msg.transforms[i].header.stamp = buffer
                outbag.write(topic, msg, msg.transforms[i].header.stamp)

        elif topic == "/tf_static" and msg.transforms:
            for i in range(len(msg.transforms)):
                msg.transforms[i].header.stamp = buffer
                outbag.write(topic, msg, msg.transforms[i].header.stamp)
        else:
            msg.header.stamp = buffer
            outbag.write(topic, msg, msg.header.stamp)
