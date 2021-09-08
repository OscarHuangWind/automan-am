import os
import cv2
import sys
sys.path.append('..')
import numpy as np
from math import cos, sin
# from moviepy.editor import *
from lib.FSANET_model import *
import numpy as np
from keras.layers import Average
# from moviepy.editor import *
# from mtcnn.mtcnn import MTCNN 

import pygame
import time
from pykinect2 import PyKinectRuntime
from pykinect2 import PyKinectV2
import struct


class KinectRecorder:
  def __init__(self, save_prefix, save_on_record):

    self.kinect = PyKinectRuntime.PyKinectRuntime(
      PyKinectV2.FrameSourceTypes_Depth |
      PyKinectV2.FrameSourceTypes_Color |
      PyKinectV2.FrameSourceTypes_BodyIndex
    )
    self.depth_height = self.kinect.depth_frame_desc.Height
    self.depth_width = self.kinect.depth_frame_desc.Width
    self.color_height = self.kinect.color_frame_desc.Height
    self.color_width = self.kinect.color_frame_desc.Width
    self.body = np.zeros([self.depth_height, self.depth_width])
    self.color = np.zeros([self.color_height, self.color_width, 3])
    self.depth = np.zeros([self.depth_height, self.depth_width])
    self.color_out = None
    self.color_frames = []
    self.depth_frames = []
    self.body_frames = []
    self.fps = 10
    self.recording = False
    self.save_prefix = save_prefix
    self.save_on_record = save_on_record
    self.min_fps = 10

  def run(self):
    """
    Main loop. Press any key to start recording. Close the window to stop
    recording.

    """
    self.color = self.kinect.get_last_color_frame()
    self.color = np.reshape(
          self.color,
          [self.color_height, self.color_width, -1]
    )
    self.color = self.color[..., :3]
    return self.color  


def draw_axis(img, yaw, pitch, roll, tdx=None, tdy=None, size = 50):
    print(yaw,roll,pitch)
    pitch = pitch * np.pi / 180
    yaw = -(yaw * np.pi / 180)
    roll = roll * np.pi / 180

    if tdx != None and tdy != None:
        tdx = tdx
        tdy = tdy
    else:
        height, width = img.shape[:2]
        tdx = width / 2
        tdy = height / 2

    # X-Axis pointing to right. drawn in red
    x1 = size * (cos(yaw) * cos(roll)) + tdx
    y1 = size * (cos(pitch) * sin(roll) + cos(roll) * sin(pitch) * sin(yaw)) + tdy

    # Y-Axis | drawn in green
    #        v
    x2 = size * (-cos(yaw) * sin(roll)) + tdx
    y2 = size * (cos(pitch) * cos(roll) - sin(pitch) * sin(yaw) * sin(roll)) + tdy

    # Z-Axis (out of the screen) drawn in blue
    x3 = size * (sin(yaw)) + tdx
    y3 = size * (-cos(yaw) * sin(pitch)) + tdy

    cv2.line(img, (int(tdx), int(tdy)), (int(x1),int(y1)),(0,0,255),3)
    cv2.line(img, (int(tdx), int(tdy)), (int(x2),int(y2)),(0,255,0),3)
    cv2.line(img, (int(tdx), int(tdy)), (int(x3),int(y3)),(255,0,0),2)

    return img
    
def draw_results_ssd(detected,input_img,faces,ad,img_size,img_w,img_h,model,time_detection,time_network,time_plot):
    
    y=0
    r=0
    p=0
    # loop over the detections
    if detected.shape[2]>0:
        for i in range(0, detected.shape[2]):
            # extract the confidence (i.e., probability) associated with the
            # prediction
            confidence = detected[0, 0, i, 2]

            # filter out weak detections
            if confidence > 0.5:
                # compute the (x, y)-coordinates of the bounding box for
                # the face and extract the face ROI
                (h0, w0) = input_img.shape[:2]
                box = detected[0, 0, i, 3:7] * np.array([w0, h0, w0, h0])
                (startX, startY, endX, endY) = box.astype("int")
                # print((startX, startY, endX, endY))
                x1 = startX
                y1 = startY
                w = endX - startX
                h = endY - startY
                
                x2 = x1+w
                y2 = y1+h

                xw1 = max(int(x1 - ad * w), 0)
                yw1 = max(int(y1 - ad * h), 0)
                xw2 = min(int(x2 + ad * w), img_w - 1)
                yw2 = min(int(y2 + ad * h), img_h - 1)
                
                faces[i,:,:,:] = cv2.resize(input_img[yw1:yw2 + 1, xw1:xw2 + 1, :], (img_size, img_size))
                faces[i,:,:,:] = cv2.normalize(faces[i,:,:,:], None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)        
                
                face = np.expand_dims(faces[i,:,:,:], axis=0)
                p_result = model.predict(face)
                
                face = face.squeeze()
                img = draw_axis(input_img[yw1:yw2 + 1, xw1:xw2 + 1, :], p_result[0][0], p_result[0][1], p_result[0][2])
                y=p_result[0][0]
                r=p_result[0][1]
                p=p_result[0][2]
                
                input_img[yw1:yw2 + 1, xw1:xw2 + 1, :] = img

    #cv2.imshow("img", input_img)
    
    return input_img, y,r,p #,time_network,time_plot

def main():
    try:
        os.mkdir('./img')
    except OSError:
        pass

    # face_cascade = cv2.CascadeClassifier('lbpcascade_frontalface_improved.xml')
    # detector = MTCNN()

    # load model and weights
    img_size = 64
    stage_num = [3,3,3]
    lambda_local = 1
    lambda_d = 1
    img_idx = 0
    detected = '' #make this not local variable
    time_detection = 0
    time_network = 0
    time_plot = 0
    skip_frame = 1 # every 5 frame do 1 detection and network forward propagation
    ad = 0.6

    #Parameters
    num_capsule = 3
    dim_capsule = 16
    routings = 2
    stage_num = [3,3,3]
    lambda_d = 1
    num_classes = 3
    image_size = 64
    num_primcaps = 7*3
    m_dim = 5
    S_set = [num_capsule, dim_capsule, routings, num_primcaps, m_dim]

    model1 = FSA_net_Capsule(image_size, num_classes, stage_num, lambda_d, S_set)()
    model2 = FSA_net_Var_Capsule(image_size, num_classes, stage_num, lambda_d, S_set)()
    
    num_primcaps = 8*8*3
    S_set = [num_capsule, dim_capsule, routings, num_primcaps, m_dim]

    model3 = FSA_net_noS_Capsule(image_size, num_classes, stage_num, lambda_d, S_set)()
    
    print('Loading models ...')

    weight_file1 = '../pre-trained/300W_LP_models/fsanet_capsule_3_16_2_21_5/fsanet_capsule_3_16_2_21_5.h5'
    model1.load_weights(weight_file1)
    print('Finished loading model 1.')
    
    weight_file2 = '../pre-trained/300W_LP_models/fsanet_var_capsule_3_16_2_21_5/fsanet_var_capsule_3_16_2_21_5.h5'
    model2.load_weights(weight_file2)
    print('Finished loading model 2.')

    weight_file3 = '../pre-trained/300W_LP_models/fsanet_noS_capsule_3_16_2_192_5/fsanet_noS_capsule_3_16_2_192_5.h5'
    model3.load_weights(weight_file3)
    print('Finished loading model 3.')

    inputs = Input(shape=(64,64,3))
    x1 = model1(inputs) #1x1
    x2 = model2(inputs) #var
    x3 = model3(inputs) #w/o
    avg_model = Average()([x1,x2,x3])
    model = Model(inputs=inputs, outputs=avg_model)
    
    # load our serialized face detector from disk
    print("[INFO] loading face detector...")
    protoPath = os.path.sep.join(["face_detector", "deploy.prototxt"])
    modelPath = os.path.sep.join(["face_detector",
        "res10_300x300_ssd_iter_140000.caffemodel"])
    net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

    # capture video
    # cap = cv2.VideoCapture('hah_color.avi')
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024*1)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768*1)
    
    p_init=0
    y_init=0
    atten_ls=[]


    cap = KinectRecorder(' ', False)
    
    print('Start detecting pose ...')
    detected_pre = np.empty((1,1,1))

    idx=0

    cv2.namedWindow('img', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('img', 640, 480)
    while True:
        # get video frame
        #ret, input_img = cap.read()
        input_img = cap.run()
        input_img=cv2.resize(input_img, (960, 540))


        img_idx = img_idx + 1
        img_h, img_w, _ = np.shape(input_img)

        
        if img_idx==1 or img_idx%skip_frame == 0:
            time_detection = 0
            time_network = 0
            time_plot = 0
            
            # detect faces using LBP detector
            gray_img = cv2.cvtColor(input_img,cv2.COLOR_BGR2GRAY)
            # detected = face_cascade.detectMultiScale(gray_img, 1.1)
            # detected = detector.detect_faces(input_img)
            # pass the blob through the network and obtain the detections and
            # predictions
            blob = cv2.dnn.blobFromImage(cv2.resize(input_img, (300, 300)), 1.0,
                (300, 300), (104.0, 177.0, 123.0))
            net.setInput(blob)
            detected = net.forward()

            if detected_pre.shape[2] > 0 and detected.shape[2] == 0:
                detected = detected_pre

            faces = np.empty((detected.shape[2], img_size, img_size, 3))

            input_img, y,r,p = draw_results_ssd(detected,input_img,faces,ad,img_size,img_w,img_h,model,time_detection,time_network,time_plot)
            #cv2.imwrite('img/'+str(img_idx)+'.png',input_img)
            
        else:
            input_img,y,r,p = draw_results_ssd(detected,input_img,faces,ad,img_size,img_w,img_h,model,time_detection,time_network,time_plot)
        

        idx+=1
        
        #prepare 
        if idx<10:
           continue
        
        #initialization 
        if p_init==0 and y_init==0:
           p_init=p
           y_init=y
           continue
        

        atten= max(isattention(abs(p-p_init),5,10),isattention(abs(y-y_init),20,30))
        atten_ls.append(atten)
        if len(atten_ls)>30:
           atten_ls.pop(0)

        state=np.mean(atten_ls)

        if state<0.3:
           state_txt='focus'
        elif state<1.3:
           state_txt='part'
        else:
           state_txt='distract'

        cv2.putText(input_img,state_txt, (320,240),cv2.FONT_HERSHEY_COMPLEX,6,(0,0,255),25)
        
        cv2.imshow("img", input_img)

        if detected.shape[2] > detected_pre.shape[2] or img_idx%(skip_frame*3) == 0:
            detected_pre = detected

        key = cv2.waitKey(1)

def isattention(pyr,t1,t2):

    if pyr>t2:
       return 2
    elif pyr>t1:
       return 1
    else:
       return 0
        
if __name__ == '__main__':
    main()