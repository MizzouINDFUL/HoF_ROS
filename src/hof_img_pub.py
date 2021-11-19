import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class HoFImgPub():

    def __init__(self):

        rospy.init_node('hof_img_pub')

        self.bridge = CvBridge()

        self.pub = rospy.Publisher("/hof_img", Image, queue_size=1)

        self.refPts = []
        self.mode = 0

        self.shape = (400, 400)
        self.img = np.zeros((self.shape[0], self.shape[1], 3), np.uint8)

        cv2.namedWindow("Raster Objects")
        cv2.setMouseCallback("Raster Objects", self.mouseCallback)


    def mouseCallback(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPts = [(x, y), (x, y)]
            self.mode = 1

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.refPts = [(x, y), (x, y)]
            self.mode = 2

        elif event == cv2.EVENT_MBUTTONDOWN:
            self.refPts = [(x, y), (x, y)]
            self.mode = 3

        elif event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_RBUTTONUP or event == cv2.EVENT_MBUTTONUP:
            self.img = self.drawRect(self.img)
            self.mode = 0
        
        elif event == cv2.EVENT_MOUSEMOVE and self.mode > 0:
            self.refPts[-1] = (x, y)


    def drawRect(self, img):
        x0, x1 = sorted([self.refPts[0][0], self.refPts[1][0]])
        y0, y1 = sorted([self.refPts[0][1], self.refPts[1][1]])

        if self.mode == 1:
            img[y0:y1, x0:x1, 2] = 255
        elif self.mode == 2:
            img[y0:y1, x0:x1, 0] = 255
        else:
            img[y0:y1, x0:x1, :] = 0

        return img

    def run(self):

        while not rospy.is_shutdown():

            tmp_img = self.img.copy()

            if self.mode > 0:
                tmp_img = self.drawRect(tmp_img)

            cv2.imshow("Raster Objects", tmp_img)

            self.pub.publish(self.bridge.cv2_to_imgmsg(tmp_img, 'bgr8'))

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                self.img = np.zeros((self.shape[0], self.shape[1], 3), np.uint8)
        
        cv2.destroyAllWindows()


if __name__ == "__main__":

    obj = HoFImgPub()
    obj.run()
    
