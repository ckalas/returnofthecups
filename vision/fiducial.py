import numpy as np
import cv2

MIN_MATCH_COUNT = 10
fid_width = 4.6 # cm

# receives the undistorted image and finds the fiducial marking
def find(img, mtx, dist):
    try:
        return siftFeature(img, mtx, dist)
    except cv2.error as e:
        print 'cv2 error in fidicuail.find'
        return (False, None)


def siftFeature(img, mtx, dist):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    id7 = cv2.imread('images/fiducial/id7.png')
    gray7 = cv2.cvtColor(id7, cv2.COLOR_BGR2GRAY)

    # start using SIFT
    sift = cv2.SIFT()

    kp, des = sift.detectAndCompute(gray,None)
    id7_kp, id7_des = sift.detectAndCompute(gray7, None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    # debug
    if des is None or id7_des is None:
        return (False, None)


    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(id7_des, des, k=2)

    # store all the good matches as per Lowe's ratio test
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([ id7_kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([ kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h, w = gray7.shape
        
        # draw a square box around the detected fiducial
        pts = np.float32([ [0,0], [0,h-1], [w-1,h-1], [w-1,0] ]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, M)
        cv2.polylines( gray, [np.int32(dst)], True, 255, 3, cv2.CV_AA)

        # Required to allow the solvePnP to work
        objp = np.zeros( (2 * 2, 3), np.float32 )
        objp[1,1:2] = 1
        objp[2,0:2] = 1
        objp[3,0:1] = 1

        # debugging messages
        # print objp
        # print 'pts: {0}, \ndst: {1}'.format(objp, dst)
        # print '\n\nM: {0}\n\n'.format(M)

        ret, rvec, tvec = cv2.solvePnP( objp, dst, mtx, dist)
        return reconfigure_reference(True, tvec, rvec)

    else:
        matchesMask = None

        return (False, None)

def reconfigure_reference(ret, tvec, rvec):
        # print '\n\nValid return values: {0}'.format( ret )

        linearScale = -4.25

        if ret:
                # xyz values are in cm units
                x, y, z = [p[0]*linearScale for p in tvec]
                x = (x-fid_width/2)
                # Adjust the origin to the xyz axis on frame
                # x, y, z = z, -(x - fid_width / 2) , -(y) 
                
                # Offset values
                #rotx, roty, rotz = rvec / np.pi * 180 
                rotx, roty, rotz = [-p[0] for p in rvec]
                rotx = (rotx-0.27)

                #print 'DEBUG: ',x,y,z,rotx,roty,rotz
                #rotx, roty, rotz = -rotz, -(rotx-16), -roty

                # change data type to allow clean output when debugging
                #x, y, z = x[0], y[0], z[0]
                #rotx, roty, rotz = rotx[0], roty[0], rotz[0]

                # Convert the rotation angle back to radian form
                #rotx, roty, rotz = rotx*(np.pi/180), roty*(np.pi/180), rotz*(np.pi/180)

                # Custom Transformation :D
                Homogenous_Transformation = np.array([
                    [np.cos(rotz)*np.cos(roty),
                    np.cos(rotz)*np.sin(roty)*np.sin(rotx)-np.sin(rotz)*np.cos(rotx),
                    np.cos(rotz)*np.sin(roty)*np.cos(rotx)+np.sin(rotz)*np.sin(rotx),
                    x],

                    [np.sin(rotz)*np.cos(roty),
                    np.sin(rotz)*np.sin(roty)*np.sin(rotx)+np.cos(rotz)*np.cos(rotx),
                    np.sin(rotz)*np.sin(roty)*np.cos(rotx)-np.cos(rotz)*np.sin(rotx),
                    y],

                    [-np.sin(roty),
                    np.cos(roty)*np.sin(rotx),
                    np.cos(roty)*np.cos(rotx),
                    z],

                    [0, 0, 0, 1] 
                    ])

                print x,y,z,rotx,roty,rotz
                return (True, Homogenous_Transformation)
        else:
                return (False, None)     
