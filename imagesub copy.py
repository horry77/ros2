import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, LaserScan, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import os, sys
from pathlib import Path


class Tb3_image_sub(Node):
    def __init__(self):
        super().__init__('tb3_image_sub')
        self.qos_profile = QoSProfile(depth = 10)
        self.create_subscription(CompressedImage, '/camera/image/compressed', self.sub_message, 10)
        self.cb = CvBridge()
        # self.traffic = cv2.imread('traffic_stop.png')
        # self.orbF = cv2.ORB_create(nfeatures=800)
        # self.bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)

    def sub_message(self, msg):
        try:
            current_frame = self.cb.compressed_imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            self.get_logger().info(e)
        
        # # 1 canny
        # canny = cv2.Canny(current_frame, 50, 100)
        # # 2 houghlineP
        # lines = cv2.HoughLinesP(canny, 1, np.pi/180.0, 100, maxLineGap=5)
        # for line in lines:
        #     x1, y1, x2, y2 = line[0]
        #     cv2.line(current_frame, (x1,y1), (x2,y2), (0,0,255), 3)
        
        # 3 inRange
        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        hsv1 = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        ret, imthres = cv2.threshold(hsv, 127, 255, cv2.THRESH_BINARY_INV)

        # 가장 바깥쪽 컨투어에 대해 꼭지점 좌표만 반환 ---④
        im2, contour2=  cv2.findContours(imthres, cv2.RETR_EXTERNAL, \
                                                cv2.CHAIN_APPROX_SIMPLE)
        for idx, cont in enumerate(im2): 
            # 랜덤한 컬러 추출 ---⑦
            # 컨투어 인덱스 마다 랜덤한 색상으로 그리기 ---⑧
            if  int(idx) <= 10:
                color = [int(i) for i in np.random.randint(0,255, 3)]
                cv2.drawContours(hsv, im2, idx, color, 3)
                # 컨투어 첫 좌표에 인덱스 숫자 표시 ---⑨
                cv2.putText(hsv, str(idx), tuple(cont[0][0]), cv2.FONT_HERSHEY_PLAIN, \
                                                                        1, (0,0,255))
        # lower = (15, 0, 0)
        # upper = (30, 255, 255)
        # hsv = cv2.inRange(hsv, lower, upper)
        # ret, mask = cv2.threshold(hsv, 100, 255,cv2.THRESH_BINARY)
        # dst = cv2.bitwise_and(current_frame, current_frame, mask=mask)

        # kp1, des1 = self.orbF.detectAndCompute(self.traffic, None)
        # kp2, des2 = self.orbF.detectAndCompute(current_frame, None)
        
        # # 4. BFMatch 
        # try:
        #     matches = self.bf.match(des1, des2)
        #     if len(matches) > 200:
        #         matches = sorted(matches, key = lambda m: m.distance)
        #         minDist = matches[0].distance
        #         good_matches = list(filter(lambda m: m.distance< 5 *minDist, matches))

        #         src1_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches])
        #         src2_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches])
        #         H, mask = cv2.findHomography(src1_pts, src2_pts, cv2.RANSAC, 3.0)
        #         h, w, _ = self.traffic.shape
        #         pts = np.float32([[0,0], [0,h-1], [w-1,h-1], [w-1,0]]).reshape(-1,1,2)
        #         pts2 = cv2.perspectiveTransform(pts, H)
        #         cv2.polylines(current_frame, [np.int32(pts2)], True, (0,0,255), 3)
        #     else:
        #         self.get_logger().info('No matches')
        # except:
        #     pass
        # print('center pixel', current_frame[300,300])

        cv2.waitKey(1) # 이미지 처리 시간 주기.
        cv2.imshow('camera', hsv)
        
    # def sub_scan(self, msg):
    #     self.get_logger().info('abcdef')
    #     print('obstacle :', msg.ranges[360])


def main(args = None):
    rclpy.init(args=args)
    node = Tb3_image_sub()
    try:
        rclpy.spin(node) # 블럭함수
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt!!')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()