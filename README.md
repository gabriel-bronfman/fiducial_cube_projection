# AR projection of Cube onto Fiducial marking
## Overview
This repository contains information on how to project a cube onto an AprilTag, or any sort of a fiducial. It has code, but also an explanation on the key components of the problem. The problem is easily solvable using integrated OpenCv functionality, but also has a self-implemented approach so that it is more transparent what to is happening.

This problem essentially boils down to three primary steps: estimating the homography of the AprilTag, extracting the orientation and position of the camera with relation to a frame attached to the AprilTag, then using that information to project points into the image plane as if you were imaging a box at the AprilTag frame.

I did this problem two ways: the easy way and the hard way. Both problems consisted initially of a detector that detected the AprilTags and returned the pixel locations of the AprilTag corners.

## Easy way

Using cv2.solvePnP, you can solve for the camera extrinsics by passing in the perceived AprilTags corners, the generally defined location of the AprilTag corners in the AprilTag frame (namely $[[0,0,0],[0,1,0],[1,1,0],[1,0,0]]$), and the intrinsics of the camera. This function is very helpful for a couple reasons. The PnP problem is actually degenerate when you have four coplanar points (like in our case). However, the function automatically reverts to estiamting the pose of the camera using homography estimation. It will return a angle-axis rotational parameterization of the orientation of the camera, along with a translation vector. These are exactly what we need to pass into our next function.

For this next section, its important to know the following equation:
```math
s\begin{bmatrix} u\\v\\1 \end{bmatrix} = K \begin{bmatrix} R|t \end{bmatrix} \begin{bmatrix} X\\Y\\Z\\1 \end{bmatrix} 
```
This is the so-called camera projection equation, and it essentially tells you, given the intrinsic and extrinsic parameters of the camera, how points in the real world are imaged in the image plane, up to a scale. This is important because, if we know where we want our cube to be in the world frame, the pose of the camera in the world frame, and our intrinsic camera parameters, we can compute the exact location of the cube as it would be imaged!

You can use projectPoints, which does this math for you, to compute the imaged locations of the imaginary cube's corners. You can then draw lines connecting the corners. Its really that simple! As you rotate the tag, you should see the cube follow the motion of the tag. 

## Hard Way

The hard way is to essentially code all the steps yourself. The first two steps use my own implementation, and while I still use projectPoints, its really just a few easy matrix equations away from your own implementation.

You can find my implementation of the DLT algorithm that, with the corners in the global frame and image plane, constructs a $Ax=0$ equation. I highly reccomend reading the Hartley-Zisserman Multiview Geometry book if you need proof why this makes sense. You can find it [here](http://www.r-5.org/files/books/computers/algo-list/image-processing/vision/Richard_Hartley_Andrew_Zisserman-Multiple_View_Geometry_in_Computer_Vision-EN.pdf) Using singular value decomposition, I can then exract the estimated homography from these points. In general, mo' points means mo' better, but since a homograhpy has 8 DOF, you need a minumum four points (i.e the four corners of the tag). To finish, I divide the entire matrix by a scalar of H[3,3] to ensure that the last element in the H matrix is 1.

For this next part, we want to remember the camera projection equation:
```math
s\begin{bmatrix} u\\v\\1 \end{bmatrix} = K \begin{bmatrix} R|t \end{bmatrix} \begin{bmatrix} X\\Y\\Z\\1 \end{bmatrix} 
```
To extract the rotation and translation from the homography, one uses a simple decomposition of the projection matrix P, defined here

```math
p = K \begin{bmatrix} R|t \end{bmatrix}
```

We know that P is equal to H, as the definition of a homography is a 2D-2D planar transformation of points that preserves only collinearity. Thus 

```math
H = K \begin{bmatrix} R|t \end{bmatrix}
```


By premultiplying the homography by $K^{-1}$, I can extract the columns $r_1 $ and $r_2$. This is confusing, but clear from the following equations:

```math
K^{-1}H = \begin{bmatrix} R|t \end{bmatrix} 
```
```math
\begin{bmatrix} u\\v\end{bmatrix} K^{-1}H   = \begin{bmatrix} r_1 & r_2 & r_3 & t \end{bmatrix} \begin{bmatrix} X\\Y\\0 \end{bmatrix}
```
```math
\begin{bmatrix} K^{-1}H \end{bmatrix}  = \begin{bmatrix} r_1 & r_2 & t \end{bmatrix} 
```
Because the we define the world frame on the fiducial, the Z coordinate for our global frame points will always be zero. This allows us to eliminate the $r_3$ vector and make the claim that  $r_1 $, $r_2$, and $t$ are simply the column vectors of H. We then normalize the column vectors of the H matrix by the average of the magnitudes $||r_1||$ and  $||r_2||$ to create vectors as close to unity as possible. 

We can simply recover the lost $r_3$ by crossing our normalized $r_1$ and $r_2$. In my implementation, I found that normalizing the three column vectors by the average of $r_1$ and $r_2$ of the resulting matrix from $K^{-1}H$ resulted in a better rotation matrix.

From my resulting R and T, I can convert my R into a angle-axis representation using cv2.Rodrigues and then pass all of that into my projectPoints, as before. I can then draw lines between my projected points.

## Comparison of approaches

The primary disadvantage to the self-implementation approach has is that I do not do a least-squared optimization on the resulting R from step 2 to find a true orthonormal rotation matrix. This means in general my results are slightly less than ideal, however still quite good.

The pictures are from both implementations:

## Optimized Way:
![alt text](https://github.com/gabriel-bronfman/project_cube/blob/main/images/easy_way_1.png "Easy Way 1")
![alt text](https://github.com/gabriel-bronfman/project_cube/blob/main/images/Easy_way_2.png "Easy Way 2")
![alt text](https://github.com/gabriel-bronfman/project_cube/blob/main/images/easy_way_3.png "Easy Way 3")

## Unoptimized Way:

![alt text](https://github.com/gabriel-bronfman/project_cube/blob/main/images/hard_way_1.png "Easy Way 1")
![alt text](https://github.com/gabriel-bronfman/project_cube/blob/main/images/hard_way_2.png "Easy Way 2")
![alt text](https://github.com/gabriel-bronfman/project_cube/blob/main/images/hard_way_3.png "Easy Way 3")
