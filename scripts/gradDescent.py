#!/usr/bin/env python3
import numpy as np
from math import sin, cos, tan, pi, sqrt, atan2

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose, Point

def Transform(R, T, p):
    trans = np.zeros((4, 4), dtype=np.float64)
    trans[:3, :3] = R
    trans[:3, 3] = T
    trans[3, 3] = 1.0
    
    res = np.matmul(trans, [p[0], p[1], p[2], 1.0])
    return [res[0]/res[3], res[1]/res[3], res[2]/res[3]]

def TranslatePoint(P, dP):
    P = np.asarray(P)
    dP = np.asarray(dP)
    
    transl = np.asarray([ P[i] + dP[i] for i in range(len(P)) ])
    return transl.flatten()

# = Rx*Ry*Rz: http://www.songho.ca/opengl/gl_anglestoaxes.html
def createRot(roll, pitch, yaw):
    return np.asarray([
        [cos(pitch)*cos(yaw),                               -cos(pitch)*sin(yaw),                                sin(pitch)],
        [cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw), cos(roll)*cos(yaw) - sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(pitch)],
        [sin(yaw)*sin(roll) - cos(roll)*sin(pitch)*cos(yaw), sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw), cos(roll)*cos(pitch)]
    ], dtype=np.float64)

def createJacobian(tp, roll, pitch, yaw):
    x, y, z = [0, 1, 2]

    Jr1 = tp[x]*0                                                     + tp[y]*0                                                         + tp[z]*0
    Jp1 = tp[x]*(-sin(pitch)*cos(yaw))                                + tp[y]*sin(pitch)*sin(yaw)                                       + tp[z]*cos(pitch)
    Jy1 = tp[x]*(-cos(pitch)*sin(yaw))                                - tp[y]*cos(pitch)*cos(yaw)                                       + tp[z]*0
    
    Jr2 = tp[x]*(-sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw)) + tp[y]*(-sin(roll)*cos(yaw) - cos(roll)*sin(pitch)*sin(yaw))     + tp[z]*(-cos(roll)*cos(pitch))
    Jp2 = tp[x]*(0                   + sin(roll)*cos(pitch)*cos(yaw)) + tp[y]*(0                   - sin(roll)*cos(pitch)*sin(yaw))     + tp[z]*(+sin(roll)*sin(pitch))
    Jy2 = tp[x]*(cos(yaw)*sin(roll)  + cos(roll)*sin(pitch)*sin(yaw)) + tp[y]*(-cos(roll)*sin(yaw) - sin(roll)*sin(pitch)*cos(yaw))     + tp[z]*0
    
    Jr3 = tp[x]*(sin(yaw)*cos(roll) + sin(roll)*sin(pitch)*cos(yaw))  + tp[y]*(cos(roll)*cos(yaw)  - sin(roll)*sin(pitch)*sin(yaw))     + tp[z]*(-sin(roll)*cos(pitch))
    Jp3 = tp[x]*(0                  - cos(roll)*cos(pitch)*cos(yaw))  + tp[y]*(0                   + cos(roll)*cos(pitch)*sin(yaw))     + tp[z]*(-cos(roll)*sin(pitch))
    Jy3 = tp[x]*(cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw))  + tp[y]*(-sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw))     + tp[z]*0

    return np.asarray([
        [Jr1, Jr2, Jr3],
        [Jp1, Jp2, Jp3],
        [Jy1, Jy2, Jy3]
    ], dtype=np.float32)

def dSquaredGradient(P, Q, R, J, t, camera_matrix):
    r,p,y = [0,1,2]
    
    fx, fy = [camera_matrix[0], camera_matrix[4]]
    cx, cy = [camera_matrix[6], camera_matrix[7]]
    
    tp = TranslatePoint(P, t)
    rtp = Transform(R, t, P)

    # U, V, W
    U = fx*(-rtp[1]) + cx*(+rtp[0])
    V = fy*(-rtp[2]) + cy*(+rtp[0])
    W = +rtp[0]

    # U = fx*(rtp[0]) + cx*(+rtp[2])
    # V = fy*(rtp[1]) + cy*(+rtp[2])
    # W = +rtp[0]

    # U derivatives
    dUdr = fx*(-J[r,1]) + cx*(+J[r,0])
    dUdp = fx*(-J[p,1]) + cx*(+J[p,0])
    dUdy = fx*(-J[y,1]) + cx*(+J[y,0])
    
    dUdtx = fx*(-R[1,0]) + cx*(+R[0,0])
    dUdty = fx*(-R[1,1]) + cx*(+R[0,1])
    dUdtz = fx*(-R[1,2]) + cx*(+R[0,2])

    # V derivatives
    dVdr = fy*(-J[r,2]) + cy*(+J[r,0])
    dVdp = fy*(-J[p,2]) + cy*(+J[p,0])
    dVdy = fy*(-J[y,2]) + cy*(+J[y,0])

    dVdtx = fy*(-R[2,0]) + cy*(+R[0,0])
    dVdty = fy*(-R[2,1]) + cy*(+R[0,1])
    dVdtz = fy*(-R[2,2]) + cy*(+R[0,2])

    # W derivatives
    dWdr = J[r,0]
    dWdp = J[p,0]
    dWdy = J[y,0]

    dWdtx = R[0,0]
    dWdty = R[0,1]
    dWdtz = R[0,2]

    # u, v
    u = U/W
    v = V/W

    # u = U/W derivatives
    dudr = (dUdr*W - U*dWdr)/(W**2)
    dudp = (dUdp*W - U*dWdp)/(W**2)
    dudy = (dUdy*W - U*dWdy)/(W**2)

    dudtx = (dUdtx*W - U*dWdtx)/(W**2)
    dudty = (dUdty*W - U*dWdty)/(W**2)
    dudtz = (dUdtz*W - U*dWdtz)/(W**2)

    # v = V/W derivatives
    dvdr = (dVdr*W - V*dWdr)/(W**2)
    dvdp = (dVdp*W - V*dWdp)/(W**2)
    dvdy = (dVdy*W - V*dWdy)/(W**2)
            
    dvdtx = (dVdtx*W - V*dWdtx)/(W**2)
    dvdty = (dVdty*W - V*dWdty)/(W**2)
    dvdtz = (dVdtz*W - V*dWdtz)/(W**2)

    return np.asarray([
        # Roll, pitch, yaw derivatives
        2*(u - Q[0])*dudr + 2*(v - Q[1])*dvdr,
        2*(u - Q[0])*dudp + 2*(v - Q[1])*dvdp,
        2*(u - Q[0])*dudy + 2*(v - Q[1])*dvdy,
        # Translation derivatives
        2*(u - Q[0])*dudtx + 2*(v - Q[1])*dvdtx,
        2*(u - Q[0])*dudty + 2*(v - Q[1])*dvdty,
        2*(u - Q[0])*dudtz + 2*(v - Q[1])*dvdtz
    ])

def gradDescent(p, q, params, camera_matrix):
    x, y, z = [0,1,2]
    roll, pitch, yaw, t = params
    
    tp = TranslatePoint(p, t)
    R = createRot(roll=roll, pitch=pitch, yaw=yaw)
    J = createJacobian(tp=tp, roll=roll, pitch=pitch, yaw=yaw)
    Grad = dSquaredGradient(p, q, R, J, t, camera_matrix)

    # lr = 1e-7
    lr = 1e-6

    # Gradient descent
    return [
        # Euler angles
        roll - lr*Grad[0],
        pitch - lr*Grad[1],
        yaw - lr*Grad[2],

        # Translation
        [
            t[0] - lr*Grad[3],
            t[1] - lr*Grad[4],
            t[2] - lr*Grad[5]
        ]
    ]

# cents_from_imgPoints: 3D centroid point of each point in base_link
# cents_from_BBs: 2D centroids of the bounding box
# prev_params: previous extrinsics
def update(cents_from_imgPoints, cents_from_BBs, prev_params, camera_matrix, iters=3):
    params = prev_params
    for i in range(iters):
        for k in range(len(cents_from_imgPoints)):
            # Update parameters
            params = gradDescent(
                p=cents_from_imgPoints[k],
                q=cents_from_BBs[k],
                camera_matrix=camera_matrix,
                params=params,
            )
 
    return params