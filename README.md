# Geometry-based Camera Calibration Using Closed-form Solution of Principal Line
 
## Goal: Solve two issues of traditional camera calibration

(i) The checkerboard patterns are usually placed randomly and used all together, without a systematic procedure to screen out ill-posed patterns

(ii) All intrinsic parameters are assumed to be fixed throughout the pattern capturing process


## Method

(1) To partly address Issue (ii), only the principal point is assumed fixed, with the skewness factor ignored and the focal length NOT assumed to be fixed.

(2) The estimation is based on the establishment of the special geometric relationship of WCS (World Coordinate System) and IPCS (Coordinate System of Image Plane).

![pic_special_geometric_relationship](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_special_geometric_relationship.png) 
![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_principal_point.png) 

The idea is if we can let the image plane and the calibration plane just remain a pitch angle, we can find:

(a) The parallel lines in x-direction on calibration plane are also paralleled in the image plane.

(b) The parallel lines in y-direction on calibration plane intersect to the vanishing point in image plane 

Then, according to the direction of ix and iy, we know if we can find a projected line in y-direction passes through the vanishing point and is also perpendicular to the projected line in x-direction, this projected line will pass through the principal point p’. We call this line “Principal Line”.  Like l4 in pi1 

So, we know one calibrated image must include one principal line in general cases. 
And once we can capture at least two images of calibration plane with different poses, the intersection of principal lines is exactly the principal point.  

+ **Finding the direction of (𝑖𝑋) and (𝑖𝑥)**
   + it’s the key for getting the special geometric relationship
   + or, equivalently, find the rotated angle theta s.t. there exists the special geometric relationship between IPCS and WCS
   ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_find_dirX.png)
   + the problem can be simplified to find the theta and (𝐴′𝐵′) or (𝐶′𝐷′) in the following figures
   ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_find_dirX_2.png)
   (a) is a rectangle in 3D real world, (b) is the same rectangle but rotated with theta degree, (c) is the corresponding camera image of (b).
   Assume the points ABCD are known, we can rotate ABCD by multiplying the rotation matrix. Also, any two images of the same planar surface in space are related by a homography.    Therefore, we can easily calculate A’B’C’ and D’ by multiplying with H to transform the rotated rectangle from WCS to IPCS.
   Then the direction of A’B’ and C’D’ can be derived by the definition of vector.
   Since we expect direction of A’B’ is parallel to C’D’ according to the rotation angle theta, so theta can be derived to arctan h7 over h8. 
   **Therefore, we can calculate the rotation angle theta by only using homography information.**
   
+ **Finding the Principal Line (and (𝑖𝑦))**
    + the principal line can be calculated by finding a line that is perpendicular to (𝐴′𝐵′) and passing through the vanishing point 𝐺′
    + again, a principal line is derived using the homography matrix alone
    ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_find_dirY.png)

+ **Derivation of the Principal Point**
    + the least squares solution
    ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_find_pp.png)
    
+ **Derivation of the Focal Length**
    + Step 1: Transform to the special geometric relationship
    ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_find_FL_1.png)
    If we want to derive the focal length in original relationship, it may suffer from there are too many unknown parameters.
    However, if we firstly transform the calibration plane and image plane to be the special geometric relationship, the rotation matrix and translation vector can be simplified 
    to R_new and T_new. Also, new Homorgraohy matrix in the special geometric relationship, could be calculated by the information of principal lines.
    
    + Step 2: Derive other intrinsic parameters by following equations
    ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_find_FL_2.png)
    Therefore, we can directly calculate focal length, the pitch angle gamma, and translation distance in z direction. 
    In the special geometric relationship, we know gamma and Tz are extrinsic parameters.
    However, if we want to calculate the extrinsic parameters in original relation, all steps are the same as Zhang’s Method. So, we will not talk about the detail.

## Closed-Form Solution
+ **Intrinsic Parameters**

![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_closeformsol_intrinsic.png)
+ **Extrinsic Parameters**

![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_closeformsol_extrinsic.png)

## Flowchart of the proposed calibration process
![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_flowchart.png)


 



 
