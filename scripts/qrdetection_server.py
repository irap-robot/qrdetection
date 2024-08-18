#!/usr/bin/env python3 

import rospy 
from sensor_msgs.msg import CompressedImage ,Image 
from cv_bridge import CvBridge
from qrdetection.srv import DetectQrcode ,DetectQrcodeResponse 
import cv2 

img_compressed = None 

def image_compress_callback(msg) : 
    global img_compressed 
    img_compressed = bridge.compressed_imgmsg_to_cv2(msg)

def handle_detect_qrcode(req) : 
    global img_compressed 
    resp = DetectQrcodeResponse()

    if img_compressed is not None : 
        ret , text, image_detect = detect_qrcode(img_compressed)

        resp.success = ret 
        resp.message = text
        resp.image = bridge.cv2_to_imgmsg(image_detect,"bgr8")

    return resp 

def detect_qrcode(img) : 
    ret ,decoded_info ,points ,straight_qrcode = qrcode_detector.detectAndDecodeMulti(img)
    # print(ret,decoded_info)

    if ret : 
        text = str(decoded_info[0])

        img_detect = cv2.polylines(img_compressed, points.astype(int), True, (0, 255, 0), 2)
        cv2.putText(img_compressed, text, points[0][0].astype(int),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        
        # image_detect_pub.publish(bridge.cv2_to_imgmsg(img_compressed,'bgr8')) 
        return ret , text ,img_detect
    else : 
        
        # image_detect_pub.publish(bridge.cv2_to_imgmsg(img_compressed,'bgr8'))  
        return ret , "" ,img 

if __name__ == "__main__" : 
    rospy.init_node("qrdetection")
    qrcode_detector = cv2.QRCodeDetector()

    bridge = CvBridge()

    detect_qrcode_service = rospy.Service("detect_qrcode",DetectQrcode,handle_detect_qrcode)

    image_compress_sub = rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage,image_compress_callback)
    image_detect_pub = rospy.Publisher("image_detected",Image,queue_size=10)

    rospy.loginfo(":: QRcode Detector is Already ::")
    rospy.spin()