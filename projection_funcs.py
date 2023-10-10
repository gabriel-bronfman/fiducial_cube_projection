import numpy as np
import cv2

def homography(input_pts, output_pts):
    
    A = np.array([], dtype=np.int64).reshape(0,9)
    
    for (ptW,ptI) in zip(input_pts,output_pts):
        
        [x,y,_] = ptW
        [xp,yp] = ptI
        
        #Contruct Ax=0 equation to solve via Singular Value Decomp
        A = np.vstack([A,np.array([[-x, -y, -1, 0, 0, 0, x*xp, y*xp, xp], 
                                   [0, 0, 0, -x, -y, -1, x*yp, y*yp, yp]])])
    
    
    U, _ , V = np.linalg.svd(A)
    
    h = np.reshape(V[-1, :], (3, 3))
    
    #After extracting the information, we normalize by h[3,3] to make h[3,3] 1
    s = h[-1,-1]
    H = h/s
    
    return h

def solveExtrinsicsFromH(h,k):
    b = (np.linalg.inv(k)@h)
    
    if np.linalg.det(b)<0:
        b =  b*(-1)
    [a1,a2,a3] = b.T
    
    #Instead of normalizing by the value of A1, or A2, we average the two and normalize by both to create the better rotation
    # matrix
    Lambda = 2/(np.linalg.norm(a1) + np.linalg.norm(a2))
    r1 = Lambda * a1
    r2 = Lambda * a2
    r3 = np.cross(r1, r2, axis = 0)
    t = Lambda * a3
    h =  np.column_stack((r1, r2, r3))
     
    return h,t
    
def solveExtrinsicsFromHwithSVD(h,k):
    b = (np.linalg.inv(k)@h)
    
    if np.linalg.det(b)<0:
        b =  b*(-1)
    [a1,a2,a3] = b.T
    
    # Extract components of rotation matrix and translation from homography

    r1 = a1
    r2 = a2
    r3 = np.cross(r1, r2, axis = 0)
    t = (1/np.linalg.norm(a1)) * a3
    h =  np.column_stack((r1, r2, r3))
    
    # Solve least-squared optimization to find closest true rotation matrix to estimated solution
    u, s , v = np.linalg.svd(h)
    
    R = u@np.array([[1,0,0],[0,1,0],[0,0,np.linalg.det(u@v.T)]])@v.T
    
    return R,t

def drawCube(frame,tag_corners, cube_corners):
    (CubePtA,CubePtB,CubePtC,CubePtD) = cube_corners
    (ptA, ptB, ptC, ptD) = tag_corners
    
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))
  
    CubePtA = (int(CubePtA[0][0]), int(CubePtA[0][1]))
    CubePtB = (int(CubePtB[0][0]), int(CubePtB[0][1]))
    CubePtC = (int(CubePtC[0][0]), int(CubePtC[0][1]))
    CubePtD = (int(CubePtD[0][0]), int(CubePtD[0][1]))

    
    cv2.circle(frame, ptA, 20, (0, 0, 255), -1)
    cv2.circle(frame, ptB, 20, (0, 255, 0), -1)
    cv2.circle(frame, ptC, 20, (255, 0, 0), -1)
    cv2.circle(frame, ptD, 20, (255, 255, 255), -1)
    
    cv2.circle(frame, CubePtA, 20, (0, 0, 255), -1)
    cv2.circle(frame, CubePtB, 20, (0, 255, 0), -1)
    cv2.circle(frame, CubePtC, 20, (255, 0, 0), -1)
    cv2.circle(frame, CubePtD, 20, (255, 255, 255), -1)

    cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
    cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
    cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
    cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

    cv2.line(frame, CubePtA, CubePtB, (0, 255, 0), 2)
    cv2.line(frame, CubePtB, CubePtC, (0, 255, 0), 2)
    cv2.line(frame, CubePtC, CubePtD, (0, 255, 0), 2)
    cv2.line(frame, CubePtD, CubePtA, (0, 255, 0), 2)

    cv2.line(frame, CubePtA, ptA, (0, 255, 0), 2)
    cv2.line(frame, CubePtB, ptB, (0, 255, 0), 2)
    cv2.line(frame, CubePtC, ptC, (0, 255, 0), 2)
    cv2.line(frame, CubePtD, ptD, (0, 255, 0), 2)
    
    return frame

    

    
    
