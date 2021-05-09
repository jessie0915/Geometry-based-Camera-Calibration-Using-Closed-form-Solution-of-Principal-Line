# Geometry-based Camera Calibration Using Closed-form Solution of Principal Line
 
## Goal: Solve two issues of traditional camera calibration

(i) The checkerboard patterns are usually placed randomly and used all together, without a systematic procedure to screen out ill-posed patterns

(ii) All intrinsic parameters are assumed to be fixed throughout the pattern capturing process


## Method

(1) To partly address Issue (ii), only the principal point is assumed fixed, with the skewness factor ignored and the focal length NOT assumed to be fixed.

(2) The estimation is based on the establishment of the special geometric relationship of WCS (World Coordinate System) and IPCS (Coordinate System of Image Plane).

![pic_special_geometric_relationship]

The idea is if we can let the image plane and the calibration plane just remain a pitch angle, we can find:

(a) The parallel lines in x-direction on calibration plane are also paralleled in the image plane.

(b) The parallel lines in y-direction on calibration plane intersect to the vanishing point in image plane 

Then, according to the direction of ix and iy, we know if we can find a projected line in y-direction passes through the vanishing point and is also perpendicular to the projected line in x-direction, this projected line will pass through the principal point p’. We call this line “Principal Line”.  Like l4 in pi1 

