import numpy as np
from matplotlib import pyplot as plt
from collections import OrderedDict
import cv2, math, os
from scipy.linalg import cholesky
from scipy.linalg import solve
from numpy.linalg import lstsq
import gc

def get_rough_PP_from_single_calib_line(cal_line, PLANE_DIM):
    '''
    Get rough PP, which is a perpendicular projected point
    of plane center coordinate in to given calibration line
    :param cal_line: calibration line
    :param PLANE_DIM: [height, width] of plane dimension
    :return: point [x,y]
    '''
    temp = get_perpend_point_on_line(cal_line, [PLANE_DIM[1]/2,PLANE_DIM[0]/2])
    return temp

def get_perpend_point_on_line(line_ori, point):
    '''
    get perpendicular projected point in to a given line from given point
    '''
    line = np.copy(line_ori)
    point = np.asarray(point).reshape(-1)
    line = np.asarray(line).reshape(-1)
    a,b,c = line
    x1 = 0; x2 = 1
    y1 = (c-a*x1)/b; y2 = (c-a*x2)/b
    u = np.array([x1, y1])
    v = np.array([x2, y2])

    # calculate projected PP to cal_line
    n = v - u
    n /= np.linalg.norm(n, 2)
    projected_point = u + n * np.dot(point - u, n)
    return projected_point

def measure_two_rotations(R1, R2):
    '''
    Measure the difference between two 3x3-rotation matrix
    '''
    R = np.dot(R1, R2.T)
    temp = np.clip((np.trace(R)-1)/2,-1,1)
    error_deg = math.acos(temp)
    error_deg = np.rad2deg(error_deg)
    return error_deg

def get_rot_mat(theta):
    '''
    Calculate GT of rotation matrix
    :param theta: [theta_x, y, z]
    :return: rotation matrix in Rodrigues format
    '''
    theta_x, theta_y, theta_z = theta
    theta_x = np.deg2rad(theta_x)
    theta_y = np.deg2rad(theta_y)
    theta_z = np.deg2rad(theta_z)
    rx_matrix = np.matrix([[1, 0, 0], [0, np.cos(theta_x), -np.sin(theta_x)], [0, np.sin(theta_x), np.cos(theta_x)]])
    ry_matrix = np.matrix([[np.cos(theta_y), 0, np.sin(theta_y)], [0, 1, 0], [-np.sin(theta_y), 0, np.cos(theta_y)]])
    rz_matrix = np.matrix([[np.cos(theta_z), -np.sin(theta_z), 0], [np.sin(theta_z), np.cos(theta_z), 0], [0, 0, 1]])
    rot_mat = rz_matrix.dot(ry_matrix).dot(rx_matrix)
    rot_rod = cv2.Rodrigues(rot_mat)[0]
    return rot_mat, rot_rod

def to_rotmat(rodri_mat):
    '''
    Convert rodrigues rotation to 3x3 rotation matrix
    '''
    rot_mat = cv2.Rodrigues(rodri_mat)[0]
    return rot_mat

def to_rodrigues(rodri_mat):
    '''
    Convert 3x3 rotation matrix to rodrigues rotation
    '''
    rod_mat = cv2.Rodrigues(rodri_mat)[0]
    return rod_mat

def get_euclidean_distance(V1, V2):
    '''
    Calculate euclidean distance between two vectors
    '''
    temp1 = np.asarray(V1).reshape((-1))
    temp2 = np.asarray(V2).reshape((-1))
    euc_dist = np.sqrt(np.sum(np.square(temp2-temp1)))
    return euc_dist

def rotate3d(point, translation, theta):
    '''
    do rotation-translation given points
    '''
    theta_x, theta_y, theta_z = theta
    theta_x = np.deg2rad(theta_x)
    theta_y = np.deg2rad(theta_y)
    theta_z = np.deg2rad(theta_z)
    rx_matrix = np.matrix([[1,0,0],[0,np.cos(theta_x),-np.sin(theta_x)],[0,np.sin(theta_x),np.cos(theta_x)]])
    ry_matrix = np.matrix([[np.cos(theta_y),0,np.sin(theta_y)],[0,1,0],[-np.sin(theta_y),0,np.cos(theta_y)]])
    rz_matrix = np.matrix([[np.cos(theta_z),-np.sin(theta_z),0],[np.sin(theta_z),np.cos(theta_z),0],[0,0,1]])
    rot_mat = rz_matrix.dot(ry_matrix).dot(rx_matrix)
    #rot_mat = rx_matrix.dot(ry_matrix).dot(rz_matrix)

    translation = np.reshape(translation, (-1, 1))
    rot_trans_mat = np.append(rot_mat, translation, axis=1)
    rot_trans_mat = np.append(rot_trans_mat,[[0,0,0,1]],axis=0)
    point = np.append(point.reshape((-1,1)),[[1]],axis=0)
    rotated_pts = np.dot(rot_trans_mat, point)
    return rotated_pts, np.array([rotated_pts[0,0], rotated_pts[1,0]])

def projection(point, F, PP, skew):
    '''
    do projection given internal parameters
    '''
    proj_mat = np.matrix([[F[0],skew,PP[0],0],[0,F[1],PP[1],0],[0,0,1,0]])
    proj_point = np.dot(proj_mat,point)
    proj_point = proj_point/proj_point[-1,0]
    proj_point = proj_point[:2,0]
    return np.asarray(proj_point).flatten()

def create_obj_pts(ver_pts, hor_pts):
    '''
    create chess-board feature points
    '''
    scaling_unit = 1
    result = np.full((ver_pts*hor_pts,3),np.float32(0))
    k=0
    for i in range(hor_pts):
        for j in range(ver_pts):
            result[k, 0] = i
            result[k, 1] = j
            k += 1
    return result * scaling_unit

def create_pts_center_zeroorigin(ver_pts, hor_pts):
    '''
    create chess-board feature points with center is in (0, 0)
    '''
    # create right-top of cartesian system points
    result = np.full((ver_pts * hor_pts, 2), np.float32(1))
    k = 0
    for i in range(hor_pts):
        for j in range(ver_pts):
            result[k, 0] = i
            result[k, 1] = j
            k += 1
    # shifting a half to left & bottom to make the origin in (0, 0)
    result = result - np.array([[(ver_pts-1)/2, (hor_pts-1)/2]])
    return result

def file_writers(WITH_NOISE, PATH_RESULT, UNIFORM_NOISE):
    '''
    to make file writers
    '''
    return_list = []
    if WITH_NOISE == False:
        # check PATH_RESULT exist
        if not os.path.exists(PATH_RESULT + '/withoutnoise'):
            os.makedirs(PATH_RESULT + '/withoutnoise')
        fn_txt_PP = PATH_RESULT + '/withoutnoise/result_withoutnoise.txt'
        fn_txt_mean = PATH_RESULT + '/withoutnoise/result_withoutnoise_mean.txt'
        fn_txt_std = PATH_RESULT + '/withoutnoise/result_withoutnoise_std.txt'
        fn_txt_zhang = PATH_RESULT + '/withoutnoise/zhang_withoutnoise.txt'
        fn_txt_FL = PATH_RESULT + '/withoutnoise/FL_withoutnoise.txt'
        fn_txt_H = PATH_RESULT + '/withoutnoise/H_withoutnoise.txt'
    else:
        if not os.path.exists(PATH_RESULT + '/withnoise'):
            os.makedirs(PATH_RESULT + '/withnoise')
        fn_txt_PP = PATH_RESULT + '/withnoise/result_withnoise' + str(UNIFORM_NOISE[1]) + '.txt'
        fn_txt_mean = PATH_RESULT + '/withnoise/result_withnoise' + str(UNIFORM_NOISE[1]) + '_mean.txt'
        fn_txt_std = PATH_RESULT + '/withnoise/result_withnoise' + str(UNIFORM_NOISE[1]) + '_std.txt'
        fn_txt_mean_mse = PATH_RESULT + '/withnoise/result_withnoise' + str(UNIFORM_NOISE[1]) + '_mean_mse.txt'
        fn_txt_std_mse = PATH_RESULT + '/withnoise/result_withnoise' + str(UNIFORM_NOISE[1]) + '_std_mse.txt'
        fn_txt_zhang = PATH_RESULT + '/withnoise/zhang_withnoise.txt'
        fn_txt_FL = PATH_RESULT + '/withnoise/FL_withnoise.txt'
        fn_txt_H = PATH_RESULT + '/withnoise/H_withnoise.txt'
        file_wrt_mean_mse = open(fn_txt_mean_mse, 'w+')
        file_wrt_std_mse = open(fn_txt_std_mse, 'w+')
        return_list.extend([file_wrt_mean_mse, file_wrt_std_mse])
    file_wrt = open(fn_txt_PP, 'w+')
    file_wrt_mean = open(fn_txt_mean, 'w+')
    file_wrt_std = open(fn_txt_std, 'w+')
    file_wrt_zhang = open(fn_txt_zhang, 'w+')
    file_wrt_FL = open(fn_txt_FL, 'w+')
    file_wrt_H = open(fn_txt_H, 'w+')
    return_list.extend([file_wrt,file_wrt_mean,file_wrt_std,file_wrt_zhang,file_wrt_FL,file_wrt_H])
    return return_list

def dist_point2line(p,line_ori):
    '''
    calculate distance of point to line
    '''
    line = np.copy(line_ori)
    a,b,c = line
    c = -c # our line is in ax+by=c, and should use ax+by+c=0 format
    x,y = p
    return np.abs(a*x+b*y+c)/np.sqrt(a*a+b*b)

def calculate_least_squared_point(lines):
    '''
    find intersection point of lines via least squared method
    :return:
        point: intersection point
        d: 1D array of distance between each line to intersection point
    '''
    p = np.copy(lines[:,0]).reshape((-1,1))
    q = np.copy(lines[:,1]).reshape((-1,1))
    D = np.concatenate((p,q), axis=1)
    R = np.copy(lines[:,2]).reshape((-1,1))
    point = np.linalg.inv((D.T).dot(D)).dot(D.T).dot(R).ravel()
    d = np.zeros(lines.shape[0],dtype=np.float32)
    for i in range(lines.shape[0]):
        d[i] = dist_point2line(point,lines[i])
    return point, d

def p2p_distance2d(p1,p2):
    '''
    calculate distance between two points
    '''
    x1=p1[0]
    y1=p1[1]
    x2=p2[0]
    y2=p2[1]
    d=np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
    return d
def draw_calibration_lines(calibration_lines_ori, index, least_square_point, n_planes,
                           filename, PLANE_DIM, legend=True):
    '''
    draw calibration lines with its intersection as PP
    :param calibration_lines: calibration lines
    :param index: when doing in noisy data with n REPEAT_TIME, select only one using this index
    :param least_square_point: estimated PP
    :param filename: filename of image to be stored
    '''
    calibration_lines = np.asarray(np.copy(calibration_lines_ori))
    calibration_lines = np.copy(calibration_lines[index*n_planes:index*n_planes+n_planes,:])
    intersection_point = np.copy(least_square_point[index,:]).ravel()
    x, y = intersection_point[0], intersection_point[1]
    x_coor = np.round(x); y_coor = np.round(y)
    plt.figure('calibration lines')
    plt.clf()
    plt.rcParams.update({'font.size': 18})
    plt.xlim(0, PLANE_DIM[1])
    plt.ylim(0, PLANE_DIM[0])
    for i in range(n_planes):
        a,b,c = calibration_lines[i,:].ravel()
        x1 = 0; x2 = PLANE_DIM[1]
        y1 = (c - a*x1) / b
        y2 = (c - a*x2) / b
        x = [x1,x2]; y = [y1,y2]
        plt.plot(x, y, zorder=i, linewidth=4, label=str(i))

    if legend is True:
        plt.legend()
    plt.scatter(intersection_point[0], intersection_point[1], s=70, c='black',zorder=n_planes+1)
    plt.text(x_coor+4, y_coor+8, '(' + str(x_coor) + ',' + str(y_coor) + ')', zorder=n_planes+1, fontsize=20, color='black')
    plt.savefig(filename+".png", bbox_inches='tight') # save as png
    plt.savefig(filename+".eps", bbox_inches='tight') # save as eps


def to_bottomleft_img_pts(pts_in, plane_h):
    '''
    convert feature points in IPCS, from top-left origin
    to bottom-left origin for our method
    :param pts_in: input points
    :param plane_h: plane height
    :return: converted pts
    '''
    pts_out = np.copy(pts_in)
    pts_out[:, 1] = plane_h - pts_out[:,1]
    return pts_out

def to_bottomleft_img_pts_zhang(pts_in, plane_h):
    '''
    convert feature points in IPCS, from top-left origin
    to bottom-left origin for zhang method
    :param pts_in: input points
    :param plane_h: plane height
    :return: converted pts
    '''
    pts_out = np.copy(pts_in)
    pts_out[:,:,1] = plane_h - pts_out[:,:,1]
    return pts_out

def plot_feature_point(rA_2d, rB_2d, rC_2d, rD_2d, IPCS_PTS, specs, PLANE_DIM):
    '''
    visualizing feature points in WCS and IPCS
    '''
    r_PTS = np.vstack((rA_2d, rB_2d, rC_2d, rD_2d))
    plt.figure('rotated image in WCS')
    for i in range(r_PTS.shape[0]):
        p1 = i % 4; p2 = (i + 1) % 4
        x = [r_PTS[p1, 0], r_PTS[p2, 0]]
        y = [r_PTS[p1, 1], r_PTS[p2, 1]]
        plt.plot(x, y)
    plt.figure('IPCS img')
    plt.rcParams.update({'font.size': 20})
    plt.clf()
    for i in range(r_PTS.shape[0]):
        p1 = i % 4; p2 = (i + 1) % 4
        x = [IPCS_PTS[p1, 0], IPCS_PTS[p2, 0]]
        y = [IPCS_PTS[p1, 1], IPCS_PTS[p2, 1]]
        plt.plot(x, y, linewidth=5)
        coordinate_string = \
            "("+str(np.around(IPCS_PTS[i,0],decimals=1)) + ",\n" \
            +str(np.around(IPCS_PTS[i,1],decimals=1))+")"
        plt.text(IPCS_PTS[i,0], IPCS_PTS[i,1], coordinate_string, fontsize=20)
    plt.axis('equal')
    plt.xlim(0, PLANE_DIM[1])
    plt.ylim(0, PLANE_DIM[0])
    plt.savefig(specs+".eps", bbox_inches='tight')


def line_plot(line, plane_dim, name, legend, color='blue'):
    '''
    plot line equation in a plane
    :param line: [a,b,c] for ax+by+c=0
    :param plane_dim: [H, W]
    :param name: title of plt.figure()
    :return:
    '''
    line_copy = np.copy(line).reshape((-1,3))
    plt.figure(name)
    for i in range(line_copy.shape[0]):
        a, b, c = line_copy[i]
        x0 = 0; x1 = plane_dim[1]
        y0 = 0; y1 = plane_dim[0]
        m_thresh = plane_dim[0] / plane_dim[1]
        m_1 = -(a / b)
        if abs(m_1) > m_thresh:
            x0 = (c / a) + (y0 / m_1)
            x1 = (c / a) + (y1 / m_1)
        else:
            y0 = (c / b) + (m_1 * x0)
            y1 = (c / b) + (m_1 * x1)
        plt.plot([x0,x1],[y0,y1],label=legend,c=color)
    plt.axis('equal')
    plt.ylim(0, plane_dim[0])
    plt.xlim(0, plane_dim[1])
    # plot only unique label
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = OrderedDict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())
    #plt.legend()


def angle_line_to_yaxispos(line):
    '''
    calculate angle (in degree, counter-clockwise) of a line w.r.t positive y axis
    '''
    a, b, c = line
    m = -a/b
    degree = np.rad2deg(np.arctan(1/m))
    return degree

def angle_of_two_lines(line1, line2):
    a1,b1,c1 = line1
    a2,b2,c2 = line2

    denominator = np.sqrt(a1 * a1 + b1 * b1)
    a1, b1, c1 = a1 / denominator, b1 / denominator, c1 / denominator  # change to unit vector

    denominator = np.sqrt(a2 * a2 + b2 * b2)
    a2, b2, c2 = a2 / denominator, b2 / denominator, c2 / denominator  # change to unit vector


    m1 = -(a1/b1); m2 = -(a2/b2)
    degree = np.rad2deg(np.arctan((m1-m2)/(1+m1*m2)))

    return degree

def detranslate(point, delta):
    '''
    Deltranslate a given point
    '''
    point = point - delta
    return point

def translate(point, delta):
    '''
    Translate a given point
    '''
    point = point + delta
    return point

def plot_4_feature_pts(pts, name):
    '''
    :param pts: 4 cols (pts) of 3x1 array (x,y,1)
    :return:
    '''
    pts = pts[:2,:].T
    plt.figure(name)
    for i in range(pts.shape[0]):
        p1 = i % 4;p2 = (i + 1) % 4
        x = [pts[p1, 0], pts[p2, 0]];
        y = [pts[p1, 1], pts[p2, 1]]
        plt.plot(x, y)

def new_coor_sys_trans(line_eq, new_origin):
    '''
    Calculate a transformation matrix for new coordninate system given new origin and new y_axis
    :param: line_eq: [a,b,c] for line equation of ax+by+c=0 as new y_axis
    :param: new_origin: new origin of new coordinate system
    :return: 3x3 transformation matrix
    '''
    a, b, c = -np.asarray(line_eq).flatten() # apparently our line eq is upsite down, then, multiply it with -1
    denominator = np.sqrt(a*a+b*b)
    a,b, c = a/denominator, b/denominator, c/denominator # change to unit vector
    theta_zz1 = np.arccos(a) * 180 / np.pi
    rot = np.array([[a, b, 0],
                    [-b, a, 0],
                    [0, 0, 1]])
    trans = np.array([[1, 0, -new_origin[0]],
                      [0, 1, -new_origin[1]],
                      [0, 0, 1]])
    H = np.dot(rot, trans)
    return H

def get_perpendicular_line(ref_line, ref_point):
    '''
    give [a,b,c] of a line ax+by+c=0 perpendicular to ref_line going thru ref_point
    :param ref_line: [a,b,c] of ax+by+c=0 reference line
    :param point: [x, y] of reference point
    :return: [a,b,c] of the perpendicular line
    '''
    a,b,c = ref_line
    x,y=ref_point
    c2 = b*x-a*y
    return np.array([b,-a,c2])
def calculate_H1_no_disp(cal_line_ori, est_PP):
    '''
    Calclate H1r x H1s in (12) of our ICCV paper
    '''
    cal_line = np.copy(cal_line_ori)
    H1 = new_coor_sys_trans(cal_line, est_PP)

    return H1

def calculate_H1(cal_line_ori, est_PP, PTS_ICPS=None):
    '''
    Calclate H1r x H1s in (12) of our ICCV paper
    '''
    cal_line = np.copy(cal_line_ori)
    H1 = new_coor_sys_trans(cal_line, est_PP)
    if PTS_ICPS is not None:
        # hom_pad = np.full((PTS_ICPS.shape[0], 1), np.float32(1))
        # PTS_ICPS = np.concatenate((PTS_ICPS, hom_pad), axis=1).astype(np.float32).T
        PTS_ICPS = np.append(PTS_ICPS.T,[[1,1,1,1]], axis=0)
        PTS_new = np.dot(H1,PTS_ICPS)
        plot_4_feature_pts(PTS_ICPS,'IPCS')
        plt.axis('equal')
        plot_4_feature_pts(PTS_new,'H1*IPCS')
        plt.axis('equal')
    line_plot(cal_line,[480, 640],'Coord. system in IPCS','y axis')
    x_line = get_perpendicular_line(cal_line, est_PP)
    line_plot(x_line, [480, 640], 'Coord. system in IPCS','x axis')
    plt.scatter(est_PP[0],est_PP[1], label='PP IPCS')
    plt.text(est_PP[0],est_PP[1],'('+str(np.around(est_PP[0], decimals=1))+', '+ \
             str(np.around(est_PP[1], decimals=1))+')')
    plt.legend(loc='upper right')
    # plt.show()
    plt.close()
    return H1, x_line

def calculate_H2(cal_line_ori, x_line, H, est_PP):
    '''
    Calclate H2r x H2s in (12) of our ICCV paper
    '''
    cal_line = np.copy(cal_line_ori)
    # caculate p0, q0 (corresponding PP in WCS)
    est_PP = np.array(est_PP).reshape((-1,1))
    est_PP = np.append(est_PP,[[1]], axis=0)
    est_PP_WCS = np.dot(np.linalg.inv(H),est_PP)
    est_PP_WCS = np.asarray(est_PP_WCS/est_PP_WCS[-1,0]).flatten()[:2]

    # calculate corresponding axis of IPCS in WCS
    cal_line[-1] = -cal_line[-1] # change to ax+by+c=0 from ax+by=-c
    cal_line = np.asarray(cal_line).reshape((-1,1))
    cal_line_WCS = np.dot(H.T,cal_line)
    x_line[-1] = -x_line[-1]
    x_line = np.asarray(x_line).reshape((-1,1))
    x_line_WCS = np.dot(H.T,x_line)

    #transform to new coordinate system
    H2 = new_coor_sys_trans(cal_line_WCS, est_PP_WCS)
    #test to plot the axis
    cal_line_WCS[-1] = -cal_line_WCS[-1]  # change back from ax+by+c=0 to ax+by=-c
    line_plot(cal_line_WCS, [640,480],'coor system in WCS','y axis')
    x_line_WCS[-1] = -x_line_WCS[-1]  # change back from ax+by+c=0 to ax+by=-c
    line_plot(x_line_WCS, [480, 640], 'coor system in WCS', 'x axis')
    plt.scatter(est_PP_WCS[0],est_PP_WCS[1],label='PP WCS')
    degree = np.abs(angle_of_two_lines(cal_line_WCS, x_line_WCS))
    plt.text(est_PP_WCS[0],est_PP_WCS[1],'('+str(np.around(est_PP_WCS[0], decimals=1))\
             +', '+str(np.around(est_PP_WCS[1], decimals=1))+')\n'\
             +'angle:'+str(degree))
    plt.legend()
    plt.close()

    #calculate angle of cal_line_WCS w.r.t y axis
    desired_rot1 = angle_line_to_yaxispos(cal_line_WCS)
    desired_rot2 = np.arctan(H[2,0]/H[2,1])
    desired_rot2_deg = np.rad2deg(desired_rot2)
    return H2, est_PP_WCS[1], cal_line_WCS, x_line_WCS

def get_WCS_axis(cal_line, est_PP, H):
    '''
    Given IPCS calibration line, estimated PP and corresponding H,
    calculate the axis in WCS
    :return
        cal_line_WCS : y_axis of WCS
        x_line_WCS: x_axis of WCS
    '''
    cal_line_copy = np.copy(cal_line)
    # get x axis of IPCS
    x_line = get_perpendicular_line(cal_line_copy, est_PP)

    # calculate corresponding axis of IPCS in WCS
    cal_line_copy[-1] = -cal_line_copy[-1]  # change to ax+by+c=0 from ax+by=-c
    cal_line_copy = np.asarray(cal_line_copy).reshape((-1, 1))
    cal_line_WCS = np.dot(H.T, cal_line_copy)
    x_line[-1] = -x_line[-1]
    x_line = np.asarray(x_line).reshape((-1, 1))
    x_line_WCS = np.dot(H.T, x_line)
    # change back from ax+by+c=0 to ax+by=-c
    cal_line_WCS[2] = -cal_line_WCS[2]
    x_line_WCS[2] = -x_line_WCS[2]
    return cal_line_WCS, x_line_WCS

def get_new_H(cal_line_ori, H, est_PP, PTS_ICPS=None):
    cal_line = np.copy(cal_line_ori)
    cal_line = np.reshape(cal_line, (-1))
    H1, x_line = calculate_H1(cal_line, est_PP, PTS_ICPS)
    H2, q0, cal_line_WCS, x_line_WCS = calculate_H2(cal_line, x_line, H, est_PP)
    new_H = np.dot(H1, H).dot(np.linalg.inv(H2))

    return new_H

def calculate_other_params(cal_line_ori, H, est_PP, index, PTS_ICPS=None):
    '''
    Calculate external parameters, focal lengts, etc
    '''
    cal_line = np.copy(cal_line_ori)
    cal_line = np.reshape(cal_line,(-1))
    H1, x_line = calculate_H1(cal_line, est_PP, PTS_ICPS)
    H2, q0, cal_line_WCS, x_line_WCS = calculate_H2(cal_line, x_line, H, est_PP)
    new_H = np.dot(H1,H).dot(np.linalg.inv(H2))
    # calculate other params
    aspectratio = 1.0
    temp = np.clip(new_H[1, 1] / (new_H[0, 0] * aspectratio), -1, 1)

    gamma = math.acos(temp)
    gamma_deg = np.rad2deg(gamma)
    c = new_H[2, 1] / math.sin(gamma)
    f = new_H[0, 0] / c
    if f < 5: # if we have big error, skip the computation as it causes singular matrix
        return new_H, 0, 0, 0, 0, 0, 0, 0, index
    tz = 1 / c
    tz_old = q0 * np.sin(gamma)
    # skew = new_H[0, 1] / c / np.cos(gamma)
    skew = new_H[0, 1] / c

    # calculate original rotation angles and translation
    M_in = np.array([[f,skew,est_PP[0]],
                    [0,f,est_PP[1]],[0,0,1]])
    print(M_in)
    M_ex = np.dot(np.linalg.inv(M_in),tz*H)

    #r3
    r3 = np.cross(M_ex[:,0], M_ex[:,1]). reshape((-1,1))
    estimated_rot_mat = np.concatenate((M_ex[:,:2],r3),axis=1)
    rod_rodri = cv2.Rodrigues(estimated_rot_mat)[0]

    trans = M_ex[:,2]
    beta = -math.asin(M_ex[2,0])
    beta_deg = np.rad2deg(beta)
    pembilang = M_ex[0,0]
    pembagi = math.cos(beta)
    temp = np.clip(pembilang/pembagi,-1,1)
    alpha = math.acos(temp)
    alpha_deg = np.rad2deg(alpha)
    temp1 = M_ex[2,1]
    temp2 = np.cos(beta)
    gamma = math.asin(temp1/temp2)
    gamma_deg = np.rad2deg(gamma)

    return new_H, f, gamma_deg, c, tz, tz_old, estimated_rot_mat, trans, index

# function to make the pattern
def plot_axis_of_symmetry2(cal_line, cal_line_WCS, x_axis_WCS,
                           desired_size, H_cam, PLANE_DIM, dxdy = (100,100)):
    '''
    Plot several lines that are parallel to WCS axis,
    and see those corresponding lines in IPCS to visualize
    that calibration line (principal line) is indeed a line of symmetry
    '''
    h, w = desired_size
    dx, dy = dxdy
    nx = (int)(w/dx); ny = (int)(h/dy)
    cal_line_WCS_norm = cal_line_WCS/cal_line_WCS[1]
    x_axis_WCS_norm = x_axis_WCS/x_axis_WCS[0]
    cal_line_WCS_norm = np.reshape(cal_line_WCS_norm,(1,-1))
    x_axis_WCS_norm = np.reshape(x_axis_WCS_norm,(1,-1))
    line_x = np.tile(x_axis_WCS_norm,(nx,1))
    line_y = np.tile(cal_line_WCS_norm,(ny,1))
    c_x = np.linspace(0,nx-1,nx); c_x -= (np.ceil(nx/2)-1)
    c_y = np.linspace(0,ny-1,ny); c_y -= (np.ceil(ny/2)-1)

    # make nx & ny numb of lines that are parallel with both axis
    # with different contant value (bias/shifting)
    for i in range(nx):
        line_x[i,2] += (c_x[i]*dx)
    for i in range(ny):
        line_y[i,2] += (c_y[i]*dy)

    # transform line in WCS to IPCS
    ## change from ax+by=c to ax+by-c=0
    line_x[:,2] = -line_x[:,2]; line_y[:,2] = -line_y[:,2]
    ## get corresponding lines in IPCS
    line_x_IPCS = np.dot(np.linalg.inv(H_cam.T),line_x.T).T
    line_y_IPCS = np.dot(np.linalg.inv(H_cam.T), line_y.T).T
    ## change back to ax+by=c from ax+by-c=0
    line_x_IPCS[:, 2] = -line_x_IPCS[:, 2]
    line_y_IPCS[:, 2] = -line_y_IPCS[:, 2]

    #plot all the lines and its calibration line as line of symmetry
    line_plot(line_x_IPCS, PLANE_DIM, 'line of sym. vis.', 'x axis wcs',
              color='red')
    line_plot(line_y_IPCS, PLANE_DIM, 'line of sym. vis.', 'y axis wcs',
              color='green')
    line_plot(cal_line, PLANE_DIM, 'line of sym. vis.', 'cal_line')


def H_synt_camera(SCALE, PP, translation, theta):
    '''
    Create equivalent H of the synthetic camera
    '''
    theta_x, theta_y, theta_z = theta
    theta_x = np.deg2rad(theta_x)
    theta_y = np.deg2rad(theta_y)
    theta_z = np.deg2rad(theta_z)
    rx_matrix = np.matrix([[1,0,0],[0,np.cos(theta_x),-np.sin(theta_x)],[0,np.sin(theta_x),np.cos(theta_x)]])
    ry_matrix = np.matrix([[np.cos(theta_y),0,np.sin(theta_y)],[0,1,0],[-np.sin(theta_y),0,np.cos(theta_y)]])
    rz_matrix = np.matrix([[np.cos(theta_z),-np.sin(theta_z),0],[np.sin(theta_z),np.cos(theta_z),0],[0,0,1]])
    rot_mat = rz_matrix.dot(ry_matrix).dot(rx_matrix)
    translation = np.reshape(translation, (-1, 1))
    rot_trans_mat = np.append(rot_mat, translation, axis=1)
    rot_trans_mat = np.append(rot_trans_mat,[[0,0,0,1]],axis=0)
    proj_mat = np.matrix([[SCALE[0], 0, PP[0], 0],
                          [0, SCALE[1], PP[1], 0],
                          [0, 0, 1, 0]])
    rot_trans_mat_new = np.delete(rot_trans_mat,2,axis=1)
    H = np.dot(proj_mat, rot_trans_mat_new)
    H_norm = H/H[2,2]
    return np.asarray(H_norm), np.asarray(H)

def remove_outliers_via_RMSE(cal_lines_ori, RMSE_THRESH, H_list):
    '''
    Remove outliers based on RMSE_THRESH
    '''
    cal_lines = np.asarray(np.copy(cal_lines_ori)).reshape((-1,3))
    point, d = calculate_least_squared_point(cal_lines)
    mask = d < RMSE_THRESH
    good_lines_idx = np.argwhere(mask==True).reshape(-1)
    good_cal_lines = cal_lines[good_lines_idx]
    point, d = calculate_least_squared_point(good_cal_lines)
    H_list = np.asarray(H_list)[good_lines_idx]
    return point, d, good_cal_lines, H_list

def getComponents(normalised_homography):
  '''((translationx, translationy), rotation, (scalex, scaley), shear)'''
  a = normalised_homography[0,0]
  b = normalised_homography[0,1]
  c = normalised_homography[0,2]
  d = normalised_homography[1,0]
  e = normalised_homography[1,1]
  f = normalised_homography[1,2]

  p = math.sqrt(a*a + b*b)
  r = (a*e - b*d)/(p)
  q = (a*d+b*e)/(a*e - b*d)

  translation = (c,f)
  scale = (p,r)
  shear = q
  theta = math.atan2(b,a)

  return (translation, theta, scale, shear)

