import os, time
import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.pyplot import cm
from utils import to_bottomleft_img_pts, calculate_other_params,\
    create_obj_pts, to_rodrigues, get_rough_PP_from_single_calib_line, \
    remove_outliers_via_RMSE


# ouliers removal using RMSE
RMSE_OURLIERS_REMOVAL = False # set True to perform outliers removal using RMSE
RMSE_THRESH = 15 # RMSE threshold to perform outliers removal

OURLIERS_REMOVAL_GAMMA = False # to do outliers removal via new_gamma
NEW_GAMMA_THRESH = 15 # new gamma threshold to perform outliers removal
MODE = 'captured_7x8'
SET_FOLDER = 'Set8'
show_all_img_with_lines = True
PATH_RESULT = "../results"
PATH_CAPTURED_PATTERN = '../data/'+MODE+'/logi/'+SET_FOLDER+'/'
PATH_CORNERS_DETECTED = PATH_RESULT + '/data/'+MODE+'/logi/'+SET_FOLDER+'/'
if not os.path.exists(PATH_CORNERS_DETECTED):
    os.makedirs(PATH_CORNERS_DETECTED)


REPEAT_TIME = 20
theta_x = theta_y = theta_z = 0
fn_txt_H_normalized = PATH_CORNERS_DETECTED + '/Homography_matrix_result_normalized.txt'
fn_txt_H_unormalized = PATH_CORNERS_DETECTED + '/Homography_matrix_result_unormalized.txt'
error_dist = PATH_CORNERS_DETECTED+'/Error_distance.txt'
file_wrt_H_normalized = open(fn_txt_H_normalized, 'w')
file_error_dist = open(error_dist, 'w')
namefile_list = []

n_clr = len(os.listdir(PATH_CAPTURED_PATTERN))
color1=iter(cm.rainbow(np.linspace(0,1,n_clr)))
color2=iter(cm.rainbow(np.linspace(0,1,n_clr)))
color3=iter(cm.rainbow(np.linspace(0,1,n_clr)))

if not os.path.exists(PATH_CORNERS_DETECTED):
    os.makedirs(PATH_CORNERS_DETECTED)
print(os.path.abspath(PATH_CORNERS_DETECTED))
if MODE == 'captured_7x8':
    PTS_VER = 6
    PTS_HOR = 7

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = create_obj_pts(PTS_VER, PTS_HOR)

DELTA = [0, 0, 4]
img = cv2.imread(os.path.join(PATH_CAPTURED_PATTERN,
                              os.listdir(PATH_CAPTURED_PATTERN)[0]),0)
PLANE_DIM = img.shape[:2]
PP = np.array([-PLANE_DIM[1] / 2, -PLANE_DIM[0] / 2])  # principal point GT: x, y
SCALE = 400
WITH_NOISE = False
UNIFORM_NOISE_MIN = -1
UNIFORM_NOISE_MAX = 1


def get_filename_in_dir(path):
    result = os.listdir(path); result = sorted(result)
    return result


def normalized_lines(lines):
    length = lines.shape[0]
    for i in range(length):
        a, b, c = lines[i]
        divide = a * a + b * b + c * c
        a /= divide
        b /= divide
        c /= divide
        lines[i] = np.array([a, b, c])


def dist_point2line(p, line):
    a, b, c = line
    x, y = p
    pembagi = np.float64(np.sqrt(a * a + b * b))
    yg_dibagi = np.float64(a * x + b * y + c)
    if a == 0 and b==0 and c ==0:
        result = 100
    else:
        result = np.abs( yg_dibagi/ pembagi)
    return result


def calculate_least_squared_point(lines):
    #normalized_lines(lines)
    A = np.dot(np.transpose(lines), lines)
    U, sigma, VT = np.linalg.svd(A)
    V = np.transpose(VT)
    x = V[0, 2] / V[2, 2]
    y = V[1, 2] / V[2, 2]
    point = np.array([x, y])
    d = np.zeros(lines.shape[0], dtype=np.float32)
    for i in range(lines.shape[0]):
        d[i] = dist_point2line(point, lines[i])
    # test using my LSE method
    p = np.copy(lines[:, 0]).reshape((-1, 1))
    q = np.copy(lines[:, 1]).reshape((-1, 1))
    D = np.concatenate((p, q), axis=1)
    R = np.copy(lines[:, 2]).reshape((-1, 1))
    point = np.linalg.inv((D.T).dot(D)).dot(D.T).dot(R).ravel()
    return point, d


def p2p_distance2d(p1, p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    d = np.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
    return d


def draw_calibration_lines(calibration_lines, least_square_point, filename, fname_ori):
    '''
    draw and write all caliration lines with their intersection
    :param calibration_lines:
    :param least_square_point:
    :param filename:
    :param fname_ori:
    :return:
    '''
    global PLANE_DIM
    #normalized_lines(calibration_lines)
    intersection_point = np.copy(least_square_point).ravel()
    x_float =intersection_point[0]; y_float = intersection_point[1]
    x_round = np.round(x_float, decimals=1); y_round = np.around(y_float, decimals=1)
    plt.figure("All calib lines")
    plt.rcParams.update({'font.size': 24})
    plt.xlim(0,PLANE_DIM[1]), plt.ylim(0,PLANE_DIM[0])
    for i in range(calibration_lines.shape[0]):
        a1, b1, c1 = calibration_lines[i, :].ravel()
        if a1 ==0 and b1==0 and c1==0:
            x1=0;x2=0;y1=0;y2=0
        else:
            x1 = -PLANE_DIM[1]; x2 = PLANE_DIM[1]
            y1 = (c1 - a1 * x1) / b1
            y2 = (c1 - a1 * x2) / b1
        x = [x1, x2];y = [y1, y2]
        plt.plot(x, y, zorder=i, c=next(color2), linewidth=3, label=str(i))
    #plt.legend()
    plt.scatter(x_float, y_float, s=50, zorder=i + 1, c='black')
    #plt.scatter(315, 233, s=100, zorder=i + 1, c='green', marker='x')
    plt.text(x_float+4, y_float+8,
             '('+str(x_round)+', '+str(y_round)+')', zorder=i+1,fontsize=24)
    #fig.subplots_adjust(bottom=0)
    #fig.subplots_adjust(top=1)
    #fig.subplots_adjust(right=1)
    #fig.subplots_adjust(left=0)
    #url = 'abcdc.com'
    plt.savefig(filename, bbox_inches='tight')
    filename = filename.replace('.png', '.eps')
    plt.savefig(filename, bbox_inches='tight')
    plt.show()


def draw_cal_lines_in_img(cal_line, img):
    a, b, c = cal_line.ravel()
    x1 = -PLANE_DIM[1]; x2 = PLANE_DIM[1]
    y1 = (c - a * x1) / b
    y2 = (c - a * x2) / b
    x1 = (int)(x1); x2=(int)(x2); y1=(int)(y1); y2=(int)(y2)
    pt1 = (x1,y1); pt2 = (x2,y2)
    cv2.line(img,pt1,pt2,(255,0,0),thickness=2)
    return img


def save_per_img_with_line(x, y, img, filename):
    '''
    write per image with its calibration line
    :param x:
    :param y:
    :param img:
    :param filename:
    :return:
    '''
    global color3
    for i in range(len(x)):
        fig_per_img = plt.figure("Per img")
        plt.clf()
        fig_per_img.subplots_adjust(bottom=0)
        fig_per_img.subplots_adjust(top=1)
        fig_per_img.subplots_adjust(right=1)
        fig_per_img.subplots_adjust(left=0)
        #plt.gca().set_axis_off()
        #plt.subplots_adjust(top=1, bottom=0, right=1, left=0,
        #                    hspace=0, wspace=0)
        #plt.margins(0,0)
        #plt.gca().xaxis.set_major_locator(NullLocator())
        #plt.gca().yaxis.set_major_locator(NullLocator())
        plt.xlim(0,PLANE_DIM[1])
        plt.ylim(0,PLANE_DIM[0])
        plt.axis('off')
        plt.imshow(img[i])
        a = np.asarray(x[i])
        b = np.asarray(y[i])
        plt.plot(a, b, c=next(color3), linewidth=14)
        filename[i] = filename[i].replace('.jpg', '.png')
        filename[i] = filename[i].replace('.JPG', '.png')
        plt.savefig(filename[i], pad_inches=0)
        plt.close(fig_per_img)

        img_with_pl = cv2.imread(filename[i], 1)
        cv2.imshow('Per img', img_with_pl)
        cv2.waitKey(20)


# main function
def calculate_PP(THETA_X):
    global PTS_HOR, PTS_VER, PATH_CORNERS_DETECTED, PATH_CAPTURED_PATTERN, namefile_list, show_all_img_with_lines
    calibration_lines = np.ndarray(shape=(0, 3), dtype=np.float32)
    N_FULLY_DETECTED = 0
    PATH_CAPTURED_PATTERN2 = PATH_CAPTURED_PATTERN
    PATH_CORNERS_DETECTED2 = PATH_CORNERS_DETECTED
    images = get_filename_in_dir(PATH_CAPTURED_PATTERN2)
    line_idx = 0
    fig2, ax2 = plt.subplots(2, 4)
    x_list = []; y_list= []; img_rgb_list = []; filename_list = []
    H_list = []; cal_lines_list = []
    for fname in images: # repeat for all img planes in folder set
        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.
        img = cv2.imread(PATH_CAPTURED_PATTERN2+'/'+fname,1)
        img_ori = np.copy(img)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        cv2.imshow("cek", gray)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (PTS_VER,PTS_HOR))

        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            corners2 = np.reshape(corners2,(-1,2))
            imgpoints.append(corners2)

            # Draw and display the corners
            img_with_corners = cv2.drawChessboardCorners(img, (PTS_VER,PTS_HOR), corners2,ret)
            # draw label on detected corners
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.3
            fontColor = (255, 255, 255)
            lineType = 1
            for i in range(corners2.shape[0]):
                cv2.putText(img_with_corners, str(i+1),
                            ((int)(corners2[i,0]),(int)(corners2[i,1])),
                            font, fontScale, fontColor, lineType)
            cv2.imwrite(PATH_CORNERS_DETECTED2+'/corner-ours_'+fname, img_with_corners)
            cv2.imshow('img_with_corders',img_with_corners)
            cv2.waitKey(20)
            N_FULLY_DETECTED += 1

            objpoints2 = np.asarray(objpoints).reshape((-1,3)).astype(np.float32)
            imgpoints2 = np.asarray(imgpoints).reshape((-1,2)).astype(np.float32)
            hom_pad = np.full((imgpoints2.shape[0],1), np.float32(1))
            imgpoints2 = np.concatenate((imgpoints2,hom_pad), axis=1).astype(np.float32)
            pts_dst = to_bottomleft_img_pts(imgpoints2, PLANE_DIM[0])
            pts_src = objpoints2
            pts_src[:,0:2] = pts_src[:,0:2]

            H, _ = cv2.findHomography(pts_src, pts_dst)
            h1, h2, h3 = H[0]
            h4, h5, h6 = H[1]
            h7, h8, h9 = H[2]

            a = (-h1 * h8 + h2 * h7)
            b = (-h4 * h8 + h5 * h7)
            c = ((h2 * h2 + h5 * h5 - h1 * h1 - h4 * h4) * h7 * h8 + (h1 * h2 + h4 * h5) * (h7 * h7 - h8 * h8)) / (
                h7 * h7 + h8 * h8)

            cal_line = np.array([[a, b, c]], dtype=np.float32)
            img_with_cal_line = draw_cal_lines_in_img(cal_line,img)

            '''
            draw cal line in image
            '''
            img_rgb = img_ori[..., ::-1]
            img_rgb_flipped = np.copy(cv2.flip(img_rgb, 0))

            x1 = -PLANE_DIM[1];x2 = PLANE_DIM[1]
            y1 = (c - a * x1) / b
            y2 = (c - a * x2) / b
            x = [x1, x2];y = [y1, y2]
            x_list.append(x); y_list.append(y); img_rgb_list.append(img_rgb_flipped); filename_list.append(PATH_CORNERS_DETECTED2+'/'+fname)

            file_wrt_H_normalized.write('pic_' + fname + ' -> h1:' + str(h1) + ', h2:' + str(h2) + ', h3:' + str(h3) +
                             ', h4:' + str(h4) +', h5:' + str(h5) +', h6:' + str(h6) +
                             ', h7:' + str(h7) +', h8:' + str(h8) +', h9:' + str(h9) +'\n\n')
            # perform ourliers removal
            if OURLIERS_REMOVAL_GAMMA == True:
                PP_rough = get_rough_PP_from_single_calib_line(cal_line, PLANE_DIM)
                _, _, gamma_deg, _, _, _, _, _, _ = \
                    calculate_other_params(cal_line,
                                           H, PP_rough,0)
                print('fname:', fname, ', gamma:', gamma_deg)
            if OURLIERS_REMOVAL_GAMMA == True and abs(gamma_deg) > NEW_GAMMA_THRESH:
                calibration_lines = np.concatenate((calibration_lines,cal_line), axis=0)
                H_list.append(H)
                line_idx += 1
            if OURLIERS_REMOVAL_GAMMA == False:
                calibration_lines = np.concatenate((calibration_lines,cal_line), axis=0)
                H_list.append(H)
                line_idx += 1

            namefile_list.append(fname)
    save_per_img_with_line(x_list,y_list,img_rgb_list,filename_list)
    PP, distance = calculate_least_squared_point(calibration_lines)
    rmse_init = np.sqrt(np.mean(np.square(distance)))

    # remove outliers via RMSE
    if RMSE_OURLIERS_REMOVAL == True:
        PP, distance, calibration_lines, H_list = \
            remove_outliers_via_RMSE(calibration_lines, RMSE_THRESH, H_list)
    return PP, distance, calibration_lines, np.asarray(H_list), N_FULLY_DETECTED

TIME_START = time.time()
PP, distance, calibration_lines, Hs, N_FULLY_DETECTED = calculate_PP(SET_FOLDER)
fl = {}; rot={}; trans={}; distance_2 = {}
singular_pattern = []; singular_H = {}; singular_cal_lines = {}
calibration_lines_copy = np.copy(calibration_lines)
for i in range(calibration_lines.shape[0]):
    gamma_deg = 'nope'
    if RMSE_OURLIERS_REMOVAL == False:# and distance[i] < RMSE_THRESH+20:
        new_H, fl_i, gamma_deg, c, tz, tz_old, rot_i, trans_i, index = \
            calculate_other_params(calibration_lines_copy[i],
                                   Hs[i], PP, index=i)
        if fl_i == 0: #if we have singular internal matrix
            singular_pattern.append(index)
            singular_H[str(index)]= Hs[i]
            singular_cal_lines[str(index)]=calibration_lines_copy[i]
        else:
            fl[str(i)] = fl_i; rot[str(i)]=rot_i; trans[str(i)]=trans_i
            distance_2[str(i)]=distance[i]
    if RMSE_OURLIERS_REMOVAL == True:
        new_H, fl_i, gamma_deg, c, tz, tz_old, rot_i, trans_i, index = \
            calculate_other_params(calibration_lines_copy[i],
                                   Hs[i], PP,i)
        fl[str(i)] = fl_i;rot[str(i)] = rot_i;trans[str(i)] = trans_i

rms = np.sqrt(np.mean(np.square(distance)))
# print('distance:', distance)
idx_best_plane=0
if RMSE_OURLIERS_REMOVAL == True:
    idx_best_plane = np.argmin(distance)
elif RMSE_OURLIERS_REMOVAL == False:
    idx_best_plane = (int)((float)(min(distance_2, key=distance_2.get)))
fl_best = fl[str(idx_best_plane)]
fl_avg = np.average(list(fl.values()))
fl_std = np.std(list(fl.values()))

print('** results from folder ' + SET_FOLDER + ' **')
print("PP:", PP)
print("RMS:", rms)
print('\nFL from best caliration line:', fl_best) # best: cal. line with least RMSE
print('Average FL:', fl_avg, ', std:', fl_std)
print("FL:", list(fl.values()))
# print rotation and translation matrix for non-singular pattern
print('\n----------------------\nTranslation\n')
for key, value in trans.items():
    print('pattern-'+key+":", list(np.around(value,decimals=1)))
print('\n----------------------\nRotation\n')
for key, value in rot.items():
    print('pattern-'+key+":", list(np.around(to_rodrigues(value),decimals=2).reshape(-1)))

filename = PATH_CORNERS_DETECTED+'PP_'+str((int)(PP[0]))+'-'+str((int)(PP[1]))+'__RMS_'+str(rms)+'.png'

# print("N_FULLY_DETECTED", N_FULLY_DETECTED)
draw_calibration_lines(calibration_lines, PP, filename, namefile_list)
print((distance.ravel()), file=file_error_dist)

#write PP to file
file_pp= open(PATH_CORNERS_DETECTED+"/PP.txt","w+")
file_pp.write(str(PP[0])+','+str(PP[1]))
file_pp.close()

TIME_STOP = time.time()
print("\n\nNeeded time:", TIME_STOP-TIME_START)