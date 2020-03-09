import rosbag
import sys
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tqdm

bridge = CvBridge()
data_dir = "/home/ANT.AMAZON.COM/sappel/kitti_data/"
output_name = data_dir + sys.argv[1] + '.bag'
input_name = data_dir + sys.argv[1] + '/synchronized_data.bag'
counter = 0
with rosbag.Bag(output_name, 'w') as outbag:
    for topic, msg, t in tqdm.tqdm(rosbag.Bag(input_name).read_messages()):
        if topic == "/kitti/camera_color_left/image_raw":
            image_path = data_dir + sys.argv[1] + "/segmented_semantic_images/" + \
                str(counter).zfill(10) + ".png"
            # print(image_path)
            image = cv2.imread(image_path)
            sem_msg = bridge.cv2_to_imgmsg(image, "bgr8")
        #outbag.write(topic, msg, msg.transforms[i].header.stamp)
            sem_msg.header = msg.header
            # print(sem_msg.header)
            counter += 1
            outbag.write("/kitti/camera_color_left/semantic", \
                sem_msg, sem_msg.header.stamp)
        elif topic == "/tf" and msg.transforms:
            for i in range(len(msg.transforms)):
                msg.transforms[i].header.stamp = t
                outbag.write(topic, msg, msg.transforms[i].header.stamp)
            continue

        elif topic == "/tf_static" and msg.transforms:
            for i in range(len(msg.transforms)):
                msg.transforms[i].header.stamp = t
                outbag.write(topic, msg, msg.transforms[i].header.stamp)
            continue
        outbag.write(topic, msg, msg.header.stamp)
