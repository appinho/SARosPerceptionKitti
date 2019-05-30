import cv2
import os

frames = 372
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255),
          (255, 128, 0), (128, 255, 0), (0, 128, 255), (0, 255, 128), (128, 0, 255), (255, 0, 128)]


# TODO: Unify filename and frame length
class tData:
    """
        Utility class to load data.
    """
    
    def __init__(self,frame=-1,obj_type="unset",truncation=-1,occlusion=-1,\
                 obs_angle=-10,x1=-1,y1=-1,x2=-1,y2=-1,w=-1,h=-1,l=-1,\
                 X=-1000,Y=-1000,Z=-1000,yaw=-10,score=-1000,track_id=-1):
        """
            Constructor, initializes the object given the parameters.
        """
        
        # init object data
        self.frame      = frame
        self.track_id   = track_id
        self.obj_type   = obj_type
        self.truncation = truncation
        self.occlusion  = occlusion
        self.obs_angle  = obs_angle
        self.x1         = x1
        self.y1         = y1
        self.x2         = x2
        self.y2         = y2
        self.w          = w
        self.h          = h
        self.l          = l
        self.X          = X
        self.Y          = Y
        self.Z          = Z
        self.yaw        = yaw
        self.score      = score
        self.ignored    = False
        self.valid      = False
        self.tracker    = -1

    def __str__(self):
        """
            Print read data.
        """
        
        attrs = vars(self)
        return '\n'.join("%s: %s" % item for item in attrs.items())

def read_tracking_results():
    root_dir = "/home/appinho/catkin_ws/src/SARosPerceptionKitti/benchmark/python/results/sha_key/data/"
    i              = 0
    s_name = "0011"
    filename       = os.path.join(root_dir, "%s.txt" % s_name)
    f              = open(filename, "r")

    f_data         = [[] for x in xrange(frames)] # current set has only 1059 entries, sufficient length is checked anyway
    ids            = []
    n_in_seq       = 0
    id_frame_cache = []
    t_data = tData()
    frame = 0
    frame_10 = str(frame).rjust(10, '0')
    img_path = '/home/appinho/kitti_data/0059/images/' + frame_10 + '.png'
    img = cv2.imread(img_path,1)
    for line in f:
        print(i)
        # KITTI tracking benchmark data format:
        # (frame,tracklet_id,objectType,truncation,occlusion,alpha,x1,y1,x2,y2,h,w,l,X,Y,Z,ry)
        line = line.strip()
        fields            = line.split(" ")
        # classes that should be loaded (ignored neighboring classes)
        classes = ["car","pedestrian"]
        if not any([s for s in classes if s in fields[2].lower()]):
            continue
        # get fields from table
        t_data.frame        = int(float(fields[0]))     # frame
        t_data.track_id     = int(float(fields[1]))     # id
        t_data.obj_type     = fields[2].lower()         # object type [car, pedestrian, cyclist, ...]
        t_data.truncation   = int(float(fields[3]))     # truncation [-1,0,1,2]
        t_data.occlusion    = int(float(fields[4]))     # occlusion  [-1,0,1,2]
        t_data.obs_angle    = float(fields[5])          # observation angle [rad]
        t_data.x1           = float(fields[6])          # left   [px]
        t_data.y1           = float(fields[7])          # top    [px]
        t_data.x2           = float(fields[8])          # right  [px]
        t_data.y2           = float(fields[9])          # bottom [px]
        t_data.h            = float(fields[10])         # height [m]
        t_data.w            = float(fields[11])         # width  [m]
        t_data.l            = float(fields[12])         # length [m]
        t_data.X            = float(fields[13])         # X [m]
        t_data.Y            = float(fields[14])         # Y [m]
        t_data.Z            = float(fields[15])         # Z [m]
        t_data.yaw          = float(fields[16])         # yaw angle [rad]
        
        if(frame != t_data.frame):
            frame_00 = str(frame).rjust(10, '0')
            cv2.imwrite("./images/" + frame_00 + '.png', img)
            frame_10 = str(t_data.frame).rjust(10, '0')
            img_path = '/home/appinho/kitti_data/0059/images/' + frame_10 + '.png'
            img = cv2.imread(img_path,1)
        x1 = (int(t_data.x1))
        x2 = (int(t_data.x2))
        y1 = (int(t_data.y1))
        y2 = (int(t_data.y2))
        cv2.rectangle(img,(x1,y1),(x2,y2), colors[t_data.track_id % 12],2)
        cv2.imshow("image", img)
        i += 1

        frame = t_data.frame

	frame_00 = str(frame).rjust(10, '0')
	cv2.imwrite("./images/" + frame_00 + '.png', img)

#read_tracking_results()

image_folder = "./images/"
video_name = 'video2.avi'

images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
print(len(images))
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape
print(height, width)

video = cv2.VideoWriter(video_name, 0, 9, (width,height))
fontFace = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 2
thickness = 3

for i,image in enumerate(sorted(images)):
    print(image)
    img = cv2.imread(os.path.join(image_folder, image))
    text = "Frame "+ str(i)
    cv2.putText(img, text, (10,50), fontFace, 2, (255,255,255), thickness, cv2.LINE_AA)
    video.write(img)

cv2.destroyAllWindows()
video.release()