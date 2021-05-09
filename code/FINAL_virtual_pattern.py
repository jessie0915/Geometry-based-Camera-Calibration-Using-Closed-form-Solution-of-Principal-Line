import time, os, cv2
import numpy as np
from utils import rotate3d, projection, file_writers, \
    calculate_least_squared_point, p2p_distance2d, draw_calibration_lines, \
    plot_feature_point, calculate_other_params, H_synt_camera, get_rot_mat,\
    to_rotmat, measure_two_rotations, get_euclidean_distance, \
    get_rough_PP_from_single_calib_line

TIME_START = time.time()
PATH_RESULT = "../results" # path to save the experiment results
CALCULATE_ZHANG = True # calculate all params of Zhang's
CALCULATE_OTHER_PARAMS = True #calculate FL and ext. params of ours
OUTLIERS_REMOVAL_EXPERIMENTS = False # set 'True' for outliers removal experiment
OUTLIERS_REMOVAL = False # set 'True' to apply outlier removal by NEW_GAMMA_THRESH
NEW_GAMMA_THRESH = 20 # threshold for outlier removal
PLOT_FEATURE_POINT = True # plot feature point in IPCS. Set this 'False' if you want to run experiment with many theta_x values, otherwise, the program will run very slow
DIFFERENT_FL_EXPERIMENT = False # set 'True' for varied focal length experiments

# noise parameters
np.random.seed(2) # create same random values in all sessions with this same random seed
WITH_NOISE = False # experiment to use noise or not
UNIFORM_NOISE = [-0.5,0.5] # [min, max] of uniform noise
REPEAT_TIME = 20 # number of repetition of experiment using noise
REPEAT_TIME_SINGLE = 1

# given camera parameters
PLANE_DIM = np.array([480,640]) # [height, width] of image plane
PP_GT = np.array([PLANE_DIM[1]/2,PLANE_DIM[0]/2]) #[x, y] of principal point
FL_GT = np.array([400, 400]) # [x, y] of focal length
n_FL_GT = np.array([[400, 400],[400, 400],
                    [400, 400],[400, 400],
                    [440, 440],[440, 440],
                    [440, 440],[440, 440]]) # will use this if DIFFERENT_FL_EXPERIMENT = True
SKEW = 0 # skewness value
TRANSLATION = [0,0,35] # [tx, ty, tz]
THETA_X = 45; THETA_Y = 0
# for theta rotation, set them directly in the for loop in the code


# four feature points in WCS for calibration
A = np.array([ -2, -2, 0], dtype=np.float32)
B = np.array([2, -2, 0], dtype=np.float32)
C = np.array([2, 2, 0], dtype=np.float32)
D = np.array([ -2, 2, 0], dtype=np.float32)
# scale the feature points in WCS
s = 4; A, B, C, D = s*A, s*B, s*C, s*D

# check PATH_RESULT exist
if not os.path.exists(PATH_RESULT):
    os.makedirs(PATH_RESULT)

# make file writers
writer_list = file_writers(WITH_NOISE,PATH_RESULT,UNIFORM_NOISE)
if WITH_NOISE == False:
    REPEAT_TIME = 1
    file_wrt_PP, file_wrt_mean, file_wrt_std, file_wrt_zhang, \
    file_wrt_FL, file_wrt_H = writer_list
    path_plane = PATH_RESULT + "/withoutnoise/IPCS_img/"
else:
    file_wrt_mean_mse, file_wrt_std_mse, file_wrt_PP, file_wrt_mean, \
    file_wrt_std, file_wrt_zhang, file_wrt_FL, file_wrt_H = writer_list
    path_plane = PATH_RESULT + "/withnoise/IPCS_img/"
if not os.path.exists(path_plane):
    os.makedirs(path_plane)

# create needed variables

# generate noise for n repeated_times, 8 coordinate axis, 8 theta_z  rotation
noise = np.random.uniform(UNIFORM_NOISE[0],UNIFORM_NOISE[1], REPEAT_TIME*8*8)
noise = np.reshape(noise, (REPEAT_TIME, 8, 8))


# start to run the main experiments
# assert not(OUTLIERS_REMOVAL == True and REPEAT_TIME > 1), \
#             "Cannot set OUTLIERS_REMOVAL True while REPEAT_TIME > 1"
assert not(OUTLIERS_REMOVAL == True and OUTLIERS_REMOVAL_EXPERIMENTS == False), \
            "Cannot OUTLIERS_REMOVAL_EXPERIMENTS == False while OUTLIERS_REMOVAL == True"

for theta_x_idx in range(1): # use this for single theta_x experiment
    theta_x = THETA_X
# for THETA_X in range(5, 50, 40): # use this for many theta_x with increment of 5
    line_idx = 0 #reset to 0, cz only store per REPEAT_TIME*8 calibration lines
    print_list = []
    print_list_our = []
    for i_time in range(REPEAT_TIME): # repeat for experiment with noise
        calibration_lines = []  # reset to 0, cz only store per REPEAT_TIME*8 calibration lines
        H_list = []
        WCS_PTS_list = []
        IPCS_PTS_list = []
        H_cam_list = []
        rot_mat_GT_list = []
        rot_mat_GT_list_ours = []
        obj_pts = []
        img_pts = []
        theta_z_idx = 0
        FL_idx = 0
        for theta_z in range(0,360,45): # repeat 8 times for rotating w.r.t z axis
            if OUTLIERS_REMOVAL_EXPERIMENTS == True: # add bad pose
                if theta_z_idx > 3: # add bad pose for planes 5-8
                    theta_y = 0; theta_x = 10
                else:
                    theta_y = THETA_Y; theta_x = THETA_X
            else: # use given THETA_Y and THETA_X for
                theta_y = THETA_Y; theta_x = THETA_X
            theta = [theta_x,theta_y,theta_z]
            if DIFFERENT_FL_EXPERIMENT ==  True:
                FL_GT = n_FL_GT[FL_idx]; FL_idx+=1
            # do 3D rotation-translation
            rA_hom, rA_2d = rotate3d(A, TRANSLATION, theta)
            rB_hom, rB_2d = rotate3d(B, TRANSLATION, theta)
            rC_hom, rC_2d = rotate3d(C, TRANSLATION, theta)
            rD_hom, rD_2d = rotate3d(D, TRANSLATION, theta)
            rot_mat_GT, rot_rodri_GT = get_rot_mat(theta)
            rot_mat_GT_list.append(rot_mat_GT)

            # do projection
            pA = projection(rA_hom, FL_GT, PP_GT, SKEW)
            pB = projection(rB_hom, FL_GT, PP_GT, SKEW)
            pC = projection(rC_hom, FL_GT, PP_GT, SKEW)
            pD = projection(rD_hom, FL_GT, PP_GT, SKEW)

            WCS_PTS = np.vstack((A[:2], B[:2], C[:2], D[:2]))
            IPCS_PTS = np.vstack((pA, pB, pC, pD))

            # calculate H camera for later purpose (visualizing line of sym.)
            H_cam_norm, H_cam_ori = H_synt_camera(FL_GT, PP_GT, TRANSLATION, theta)
            H_cam_list.append(H_cam_norm)

            tx, ty, tz = TRANSLATION; trans = str(tx) + "_" + str(ty) + "_" + str(tz)

            # add noise to feature points in IPCS for experimental with noise
            if WITH_NOISE == True:
                pA[0] += noise[i_time, theta_z_idx, 0]
                pA[1] += noise[i_time, theta_z_idx, 1]
                pB[0] += noise[i_time, theta_z_idx, 2]
                pB[1] += noise[i_time, theta_z_idx, 3]
                pC[0] += noise[i_time, theta_z_idx, 4]
                pC[1] += noise[i_time, theta_z_idx, 5]
                pD[0] += noise[i_time, theta_z_idx, 6]
                pD[1] += noise[i_time, theta_z_idx, 7]
                IPCS_PTS = np.vstack((pA, pB, pC, pD))

            if PLOT_FEATURE_POINT == True and OUTLIERS_REMOVAL_EXPERIMENTS == False:
                # visualizing feature points in IPCS
                plot_feature_point(rA_2d, rB_2d, rC_2d, rD_2d, IPCS_PTS,
                                   path_plane+"theta_x_"+str(theta_x)+"_y_"+str(theta_y) \
                                   +"_trans_"+trans+"_plane_" \
                                   +str(theta_z_idx+1), PLANE_DIM)

            if CALCULATE_ZHANG == True:
                obj_pts.extend([A,B,C,D])  # for zhang's method purpose
                img_pts.extend([pA,pB,pC,pD])  # for zhang's method purpose

            # calculate homography
            H, _ = cv2.findHomography(WCS_PTS, IPCS_PTS)

            # calculate calibration lines
            h1,h2,h3 = H[0]
            h4,h5,h6 = H[1]
            h7,h8,h9 = H[2]
            a = (-h1*h8 + h2*h7)
            b = (-h4*h8 + h5*h7)
            c = ((h2*h2+h5*h5-h1*h1-h4*h4)*h7*h8 + (h1*h2+h4*h5)*(h7*h7-h8*h8))/(h7*h7+h8*h8)
            cal_line = a, b, c

            # perform ourlier removal
            gamma_deg=0
            if OUTLIERS_REMOVAL == True:
                PP_rough = get_rough_PP_from_single_calib_line(cal_line, PLANE_DIM)
                # calculate new_gamma in our aligned coordinate system
                _, _, gamma_deg, _, _, _, _, _, _ = \
                    calculate_other_params(cal_line, H, PP_rough, IPCS_PTS)

            # take the data whose gamma_deg is above NEW_GAMMA_THRESH
            if OUTLIERS_REMOVAL == True and gamma_deg > NEW_GAMMA_THRESH:
                calibration_lines.append(cal_line)
                H_list.append(H)
                WCS_PTS_list.append(WCS_PTS)
                IPCS_PTS_list.append(IPCS_PTS)
                rot_mat_GT_list_ours.append(rot_mat_GT)
                if PLOT_FEATURE_POINT == True:
                    # visualizing feature points in IPCS
                    plot_feature_point(rA_2d, rB_2d, rC_2d, rD_2d, IPCS_PTS,
                                       path_plane + "theta_x_" + str(theta_x) + "_y_" + str(theta_y) \
                                       + "_trans_" + trans + "_plane_" \
                                       + str(theta_z_idx + 1), PLANE_DIM)
            # take all data when OUTLIERS_REMOVAL is False
            elif OUTLIERS_REMOVAL == False:
                calibration_lines.append(cal_line)
                H_list.append(H)
                WCS_PTS_list.append(WCS_PTS)
                IPCS_PTS_list.append(IPCS_PTS)
                rot_mat_GT_list_ours.append(rot_mat_GT)
                if PLOT_FEATURE_POINT == True:
                    # visualizing feature points in IPCS
                    plot_feature_point(rA_2d, rB_2d, rC_2d, rD_2d, IPCS_PTS,
                                       path_plane + "theta_x_" + str(theta_x) + "_y_" + str(theta_y) \
                                       + "_trans_" + trans + "_plane_" \
                                       + str(theta_z_idx + 1), PLANE_DIM)
            line_idx += 1
            theta_z_idx += 1

        print('theta x:', theta_x, ', repeat time:', i_time+1)

        # calculate cam params using Zhang's method
        if CALCULATE_ZHANG == True:
            img_pts = np.array(img_pts, dtype=np.float32).reshape((-1,4,2))
            obj_pts = np.array(obj_pts, dtype=np.float32).reshape((-1,4,3))
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts,
                                                               (PLANE_DIM[1],PLANE_DIM[0]),
                                                               None, None) #img size in (w,h)
            # calculate error of rotation and translation
            print('\n------------------\nZhang\'s method:')
            delta_error_rot_zhang = 0; delta_error_trans_zhang = 0
            for vecs_idx in range(len(rvecs)):
                # calculate rotation error
                rvecs_i = to_rotmat(rvecs[vecs_idx])
                error_rot = measure_two_rotations(rot_mat_GT_list[vecs_idx], rvecs_i)
                print('error_rot plane-' + str(vecs_idx+1) + ": ",
                      np.around(error_rot,decimals=2))
                delta_error_rot_zhang += error_rot

                # calculate translation error
                error_trans = get_euclidean_distance(TRANSLATION, tvecs[vecs_idx])
                print('error_trans plane-' + str(vecs_idx+1) + ": ",
                      np.around(error_trans,decimals=2))
                delta_error_trans_zhang += error_trans
            delta_error_trans_zhang = delta_error_trans_zhang / len(tvecs)
            delta_error_rot_zhang = delta_error_rot_zhang / len(rvecs)
            print('avg. delta error rot:', delta_error_rot_zhang)
            print('avg. delta error trans:', delta_error_trans_zhang)
            print('\nIntrinsic matrix:\n', mtx)

            float_formatter = lambda x: "%.2f" % x
            np.set_printoptions(formatter={'float_kind': float_formatter})
            file_wrt_zhang.write('intrinsic params, with theta x:'+str(theta_x)+'\n'\
                                 +str(mtx)+'\n-------------\n')
            obj_pts = []; img_pts = []

        # calculate PP using the proposed method
        least_square_point = np.zeros((REPEAT_TIME_SINGLE, 2), dtype=np.float32)
        if OUTLIERS_REMOVAL == False:
            idx_CL = 8
        else:
            idx_CL = len(calibration_lines)
        distance = np.zeros((REPEAT_TIME_SINGLE, idx_CL), dtype=np.float32)
        error_to_avg20 = np.zeros(REPEAT_TIME_SINGLE, dtype=np.float32)
        error_to_GT = np.zeros(REPEAT_TIME_SINGLE, dtype=np.float32)
        for i in range(REPEAT_TIME_SINGLE):
            # assert not (OUTLIERS_REMOVAL == True and REPEAT_TIME_SINGLE > 1), \
            #     "Cannot set OUTLIERS_REMOVAL True while REPEAT_TIME > 1"
            if OUTLIERS_REMOVAL == False:
                idx_CL = 8
            else:
                idx_CL = len(calibration_lines)
            calibration_lines_copy = np.asarray(np.copy(calibration_lines))
            least_square_point[i], distance[i] = calculate_least_squared_point(
                calibration_lines_copy[i * idx_CL:(i + 1) * idx_CL])
        PP_AVG = np.mean(least_square_point, axis=0)

        # calculate PP error
        for i in range(REPEAT_TIME_SINGLE):
            error_to_avg20[i] = p2p_distance2d(PP_AVG, least_square_point[i])
            error_to_GT[i] = p2p_distance2d(PP_GT, least_square_point[i])
        rmse = np.sqrt(np.mean(np.square(distance), axis=1))
        mean_rmse = np.sqrt(np.mean(rmse))
        std_rmse = np.std(rmse)

        # draw calibration lines and its intersection as PP
        index = 0  # which one to choose when doing repeating experiments with noises
        if WITH_NOISE == True:
            filename = PATH_RESULT + "/withnoise/" + "pp_withnoise" + str(UNIFORM_NOISE[1]) \
                       + "_thetax" + str(theta_x)
            index = np.argmin(error_to_GT)  # argmin to choose the best, vice versa
        else:
            filename = PATH_RESULT + "/withoutnoise/" + "pp_withoutnoise_thetax" + str(theta_x)
        draw_calibration_lines(calibration_lines, index, least_square_point,
                               idx_CL, filename, PLANE_DIM, legend=False)

        # visualize cam. calibratios as line of symmetry
        '''
        idx = 0
        cal_line_WCS, x_line_WCS = get_WCS_axis(calibration_lines[idx],
                                                PP_AVG, H_list[idx])
        plot_axis_of_symmetry2(calibration_lines[idx], cal_line_WCS,
                               x_line_WCS, [5000,6000], H_cam_list[idx],
                               PLANE_DIM, dxdy=(200,200))
        '''

        # estimate other camera params other than PP
        if CALCULATE_OTHER_PARAMS == True:
            fl = np.zeros(idx_CL)
            delta_error_trans_ours = 0
            delta_error_rot_ours = 0
            n_planes = idx_CL
            print('\n---------------------\nOurs:')
            for idx_ours in range(n_planes):
                # if idx_ours < 6:
                #     continue
                error_rot_ours = 0
                new_H, fl[idx_ours], gamma_deg, c, tz_est_our, tz_old, rot_ours, trans_ours, idx_plane = \
                    calculate_other_params(calibration_lines[idx_ours],
                                           H_list[idx_ours], PP_AVG, IPCS_PTS_list[idx_ours])

                # calculate error of rotation and translation
                try:
                    error_rot_ours = measure_two_rotations(rot_mat_GT_list_ours[idx_ours],
                                                           rot_ours)
                except:
                    print("Exception in measure_two_rotations")

                error_trans_ours = get_euclidean_distance(TRANSLATION, trans_ours)
                delta_error_rot_ours += error_rot_ours
                delta_error_trans_ours += error_trans_ours
                print('error_rot plane-' + str(idx_ours + 1) + ": ",
                      np.around(error_rot_ours, decimals=2))
                print('error_trans plane-' + str(idx_ours + 1) + ": ",
                      np.around(error_trans_ours, decimals=2))
            delta_error_rot_ours = delta_error_rot_ours / n_planes
            delta_error_trans_ours = delta_error_trans_ours / n_planes
            print('avg. delta error rot:', delta_error_rot_ours)
            print('avg. delta error trans:', delta_error_trans_ours)
            print('PP: ', PP_AVG)
            print('All FL (all planes):', fl)
            print('RMSE:', rmse)
            print('All FL (all planes) =', fl, file=file_wrt_FL)

            # plot to ivestigate axis of symmetry
            # dummy = plot_axis_of_symmetry(H_list[idx], PLANE_DIM)

        # write to file
        print('\n----------------------------------', file=file_wrt_PP)
        print('theta x:' + str(theta_x) + ' ** PP **', file=file_wrt_PP)
        print("Derived PP:", PP_AVG, file=file_wrt_PP)
        print("PP GT:", PP_GT, file=file_wrt_PP)
        print('** error to mean of 20 experimental result **', file=file_wrt_PP)
        print('mean =', np.mean(error_to_avg20), file=file_wrt_PP)
        print('std  =', np.std(error_to_avg20), file=file_wrt_PP)
        print('max  =', np.max(error_to_avg20), file=file_wrt_PP)
        print('min  =', np.min(error_to_avg20), file=file_wrt_PP)
        print('theta x:' + str(theta_x) + ' ** error to PP GT **', file=file_wrt_PP)
        print('mean =', np.mean(error_to_GT), file=file_wrt_PP)
        print('std  =', np.std(error_to_GT), file=file_wrt_PP)
        print('max  =', np.max(error_to_GT), file=file_wrt_PP)
        print('min  =', np.min(error_to_GT), ', experiment_'
                + str(np.argmin(error_to_GT) + 1) + '-ith', file=file_wrt_PP)
        file_wrt_mean.write(str(np.mean(error_to_GT)) + ", ")
        file_wrt_std.write(str(np.std(error_to_GT)) + ", ")
        if WITH_NOISE == True:
            file_wrt_mean_mse.write(str(mean_rmse) + ", ")
            file_wrt_std_mse.write(str(std_rmse) + ", ")



TIME_STOP = time.time()
print("\n\nNeeded time:", TIME_STOP-TIME_START)