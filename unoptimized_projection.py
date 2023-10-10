import cv2
import apriltag
import numpy as np
from projection_funcs import homography, solveExtrinsicsFromH, solveExtrinsicsFromHwithSVD, drawCube

cam = cv2.VideoCapture(1)

#Detector for class "36h11" AprilTags, can be changed for any type of AprilTag
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

#K matrix for my own camera, change for your own camera, my camera has no radial distortion effects but they can be added
k = np.array([[2063.8820747760683, 0, 1184.6568232403756],
             [0, 2060.440690811184, 763.9115119061984],
             [0,0,1]])
dist_coeffs = np.zeros((5, 1), np.float32)

#Define corners of the tag in the global frame. Units don't matter as all distances will be defined relative to the perceived size of the AprilTag, so we use 1 for convenience
corner_tag_global_frame = np.array([[0,0,0],[0,1,0],[1,1,0],[1,0,0]],dtype = np.float64)
corner_tag_pixel = []

if not cam.isOpened():
    print("Fail")
    exit(0)


while True:
    ret, frame = cam.read()
    
    if not ret:
        print("Failed while live")
        break
    
    
    if cv2.waitKey(1) == ord('q'):
        break
        
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    results = detector.detect(grey)
    
    for r in results:
        #Extract Corners
        corner_tag_pixel = r.corners
        
        #Solve homography for seen tag, then extract camera extrinsic parameters from seen homography
        h = homography(corner_tag_global_frame,corner_tag_pixel)
        rotation,tvec = solveExtrinsicsFromH(h,k)
        
        # Can enable this line to activate least-squared optimization approach. Will reduce stable accuracry of R but improve quality when estimated correctly
        #rotation,tvec = solveExtrinsicsFromHwithSVD(h,k)
        
        #Convert rotation matrix to Angle-Axis representation via Rodrigues formula
        rvec,_ = cv2.Rodrigues(rotation)
        
        #Project points into image plane as if there was an imaged cube above the april tag using
        # [u,v] = K[R|t][X,Y,Z]
        cube_points,_ = cv2.projectPoints(np.array([[0,0,-1],[0,1,-1],[1,1,-1],[1,0,-1]],
                                                                         dtype =np.float64), rvec,tvec,k,dist_coeffs)
        #Draw Cube
        frame = drawCube(frame,corner_tag_pixel,cube_points)
        
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
    
 
    cv2.imshow("video", frame)
cam.release()
cv2.destroyAllWindows()
        
