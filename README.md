# Geometry-based Camera Calibration Using Closed-form Solution of Principal Line
 
 https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line
 
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

## Experiments

### Synthetic Data

+ Synthetic Data without Noise
    
    In experiments, correctness of analytic expressions for the principal point will first be verified using synthetic data described in the following.
    + 𝛾=45°,𝛽=0°, ∆𝛼=45°
    + PP: image center
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_synthetic_1.png)
    + For simplicity, It is assumed that the camera optical axis is passing through the origin of WCS whose X-Y plane has a fixed rotation angle 𝛾 with respect to direction of iX and a fixed rotation angle 𝛽 with respect to direction of iY , with additional calibration planes obtained by rotating the plane, each time by ∆𝛼, with respect to direction of iZ.
    + Figure (a) shows a set of images obtained for 𝛾=∆𝛼=45° and 𝛽= 0° with the principal point located exactly at the image center, wherein four corners of a square calibration pattern are used to derive elements of H for each (virtual) 640x480 image.
    + Under the noise-free condition, Figure (b) shows the resultant eight (perfect) principal lines obtained in closed-form solution, which intersect exactly at the principal point. Also, Perfect estimation results for focal length (FL) and other extrinsic parameters are obtained for both the proposed and Zhang’s method for this simple setting.

+ Synthetic Data with Noise and Variation of Pose

    To investigate the robustness of the proposed approach, noises are added to the point features used to find Homography matrix, which are the only source of interference that may affect the correctness of each principal line.
    + Noise: uniformly distributed ±1.0 𝑝𝑖𝑥𝑒𝑙
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_synthetic_2.png)
    + The bottom-left figure illustrates calibration results similar to that shown in the previous result with noises uniformly distributed between -1 to +1 pixels added to x and y coordinates of corner points to simulate point feature localization errors resulted from image digitization. As we can see, the additive noises are lead to errors for estimating principal point.
    + For more systematic error analysis, and also taking into account the influence from different poses of the calibration plane, similar simulations are performed for two different noise levels with gamma ranging from 5° to 85° (with  delta gamma = 5° and  delta alpha= 45°), and repeated 20 times for each pose of the calibration plane.
As we can see, larger noises will result in less accurate calibration results which are also less robust.
    + Aside from the statistical comparison, we found that better results are generated for poses of calibration plane away from two degenerated conditions.
    + Moreover, according to this results, it is reasonable to suggest that: (i) the best values of  gamma are around 45°. 
    + Furthermore, as the principal point is derived via least square solution (9) from all the principal lines, it is also suggested that: (ii) it is better to distribute alpha uniformly between 0° and 180°.

+ Full Camera Calibration for _Fixed Focal Length_
    
    As for a more complete error analysis for the estimation of all parameters of a camera with fixed focal length, more general WCS-IPCS configurations (also with  gamma= 45°) are considered, which correspond to the first two datasets listed in Table I, with additive noises between positive 1.0 and negative 1.0 pixel.
    ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_synthetic_3.png)
    + One can see that smaller estimation errors are achieved for all parameters by the proposed approach.
    + As for the robustness of camera calibration, it is possible for the proposed geometric-based approach to improve the parameter estimation by screening out calibration pattern with bad poses.
    + For example, the 3rd dataset in Table I is obtained by replacing four input patterns of the 2nd dataset with four unfavorable ones (e.g., gamma < 20°), resulting in less accurate estimates for most parameters. However, by removing such patterns via the evaluation of gamma, the overall calibration results are greatly improved, as shown in the last row of Table I.

+ Full Camera Calibration for _Varied Focal Length_

    As one of the key feature, the proposed method can calibrate cameras with non-fixed focal length (FL), while Zhang’s method is not designed to cope with such situation.
Table II shows the calibration results obtained with the proposed approach, as well as those from Zhang’s method.
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_synthetic_4.png)
    + It is readily observable that even under the noise-free condition, significant estimation errors can already be observed in the latter, although perfect estimations are achieved with the proposed approach.
    + As for the noisy case, both methods generate estimation errors similar to those shown in Table I, except for the much worse estimation of focal length obtained with Zhang’s method.
    + In either case, the major difference of calibration performance is in the estimation of focal length, which is greatly constrained by the ability to cope with varied focal length during the image capturing process.
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_synthetic_5.png)
    + Table III shows additional experimental results to further evaluate the capability of the proposed approach with respect to focal length variance and the image count for each focal length. 
    + Datasets 6A and 6B are similar to dataset 6 but with additional 10% and 20% increases in the differences in focal length. Dataset 6C only consists of four images with each image having a different focal length
    + It is readily observable that satisfactory calibration results similar those obtained in Table II can be obtained with the proposed approach with (i) larger variance in the focal length and (ii) less calibration patterns for each focal length, while this is not true for Zhang’s method.
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_synthetic_6.png)
    + The above concept can be observed clearly from the geometry of principal lines of calibration results listed in Table III, as shown in Figure 6, wherein each color of a principal line corresponds to a different focal length. Note, here we assume the principal point will not change while changing the focal length.

### Real Data

For performance evaluation of the proposed calibration method under realistic conditions, 7x8 checkerboard images are employed as calibration patterns.
We demonstrate the flexibility of the proposed work over Zhang’s method by comparing their performance under two different experimental setups.
The first is using fixed focal length and another is using different focal lengths in the image capture process.
As suggested in paper, a good set of images should (i) have  gamma close to 45° and (ii) alpha should be nearly uniformly distributed between 0° and 180°.

+ Real Data with Fixed Focal Length
    + Logitech C920HD Pro camera 
    + image resolution: 640 x 480
    + 𝛾 ≅ 45°
    + 𝛼 are uniformly distributed in [0°,180°]
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_real_1.png)
    + Figure 7(a) shows eight images thus obtained with a Logitech C920HD Pro camera with a image resolution of 640x480, while Figure 7(b) shows a total of 8 principal lines, with (313.5, 246.3) being estimated as the location of the principal point.
    + Calibration result
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_real_2.png)
    + The estimated principal point, which is near the center of the image, along with the estimated focal length (FL) are shown as the data of this Table. Under the near ideal conditions, both our method and Zhang’s method produce similar results.

+ Real Data with Fixed Focal Length (Add ill-posed patterns)
    
    To evaluate the sensitivity of calibration to unfavorable (or ill-posed) calibration patterns, results of three more datasets (Sets 8-10) are also included in Table IV by replacing some good patterns in Set 7 with unfavorable ones, which can be visualized easily by comparing Figure 7(b) and Figures 8(a)-(c).
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_real_3.png)
    + The adverse influence of such replacements are readily observable from the significant deviations from the near ideal calibration results for Zhang’s method, while the proposed approach seems to be more robust as the estimated values of PP and FL are much less affected.
    + On the other hand, it is possible for the proposed approach to remove the above ill-posed calibration patterns, similar to that performed for synthetic data.
    + Specifically, Sets 11 to13 in Table V are obtained by simply screening out possible outliers whose RMSEs are greater than 15 in Sets 8 to 10. One can see that most estimations are improved, with both RMSE and STD reduced significantly. Figures 8(d)-(f) show such outlier removal results.

+ Real Data with Fixed Focal Length
    + Violate (ii) distribute 𝛼 uniformly between [0°,180°]
    + Or touch degenerate configuration in Zhang’s method
    + Beside ill-posed calibration patterns, a set of individually good patterns which violates guideline (ii) mentioned in paper may also result in unreliable estimation results.
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_real_4.png)
    + As shown in Figures 9(a) and (b) for an example of such calibration patterns and their corresponding principal lines, respectively, wherein extraneous estimation error of PP can be found in the vertical direction.
    + However, such condition can be easily detected by examining the angular extent of the principal lines, and additional calibration images can be retaken to generate principal lines of different orientation, and more reliable calibration results.

+ Real Data with _Varied Focal Length_

    In this experiment, calibration of cameras with varied focal length (FL) is considered.
    + Canon EOS 5D Mark III camera
    + image resolution: 3840 x 2560
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_real_5.png)
    + Table VI shows the calibration results for three sets of calibration patterns which are captured by a Canon EOS 5D Mark III camera with an image resolution of 3840x2560. As we can see the satisfactory calibration results are obtained with both methods for the first two datasets (Sets 14 and 15), each established for a fixed (but different) FL.
    + On the other hand, consider the last dataset (Set 16) in Table VI, which corresponds to a mixture of calibration patterns from Set 14 and Set 15, with half of them.
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_real_6.png)
    + The images of set 16 are shown in Fig. 10(a). Figures 10(b) and (c) show the principal lines calculated using all images in (a) and only using images with FL=39 mm, respectively.
    + ![](https://github.com/jessie0915/Geometry-based-Camera-Calibration-Using-Closed-form-Solution-of-Principal-Line/blob/master/picture/pic_exp_real_7.png)
    + It is readily observable from Table VI that our approach still performs satisfactorily and generates results similar to those for Sets 14 and 15.
    + However, significantly worse results are obtained with Zhang’s method which assumed fixed FL. In particular, the estimated FL (3953.0) is quite different from the two FLs obtained for the fixed cases (3822.3 and 4712.5).

# How to use

## Synthetic Data

Step 1. open `./code/FINAL_virtual_pattern.py`

Step 2. change parameters for your application (the default values can be run well)

Step 3. run the program


## Real Data

Step 1. take _n_ x _m_ chessboard pictures in the folder with path `./data/captured_nxm/logi/forder_name`, for example, `./data/captured_7x8/logi/Set8`

Step 2. open `./code/FINAL_real experiment.py`

Step 3. change parameters for your application (the default values can be run well for 7x8 chessboard)

```py
MODE = 'captured_nxm'
SET_FOLDER = 'Set8'
show_all_img_with_lines = True
PATH_RESULT = "../results"
PATH_CAPTURED_PATTERN = '../data/'+MODE+'/logi/'+SET_FOLDER+'/'
PATH_CORNERS_DETECTED = PATH_RESULT + '/data/'+MODE+'/logi/'+SET_FOLDER+'/'

if MODE == 'captured_7x8':
    PTS_VER = 6
    PTS_HOR = 7
```

Step 4. run the program
