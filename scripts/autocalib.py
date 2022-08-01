#!/usr/bin/env python3

from ccat.srv import CalibReq, CalibReqResponse
import rospy
import gradDescent


def calib(req):
    print("There are ", len(req.obsCentroids), " matchings.")
    print("Translation", req.translation)
    print("Rotation", req.euler_angles)
    print("Cam matrix", req.camera_matrix)
    
    obsCentroids = [[p.x, p.y, p.z] for p in req.obsCentroids]
    bbsCentroids = [[p.x, p.y] for p in req.bbsCentroids]

    newParams = gradDescent.update(
        cents_from_imgPoints=obsCentroids,
        cents_from_BBs=bbsCentroids,
        prev_params=[req.euler_angles[0], req.euler_angles[1], req.euler_angles[2], req.translation],
        camera_matrix=req.camera_matrix,
        iters=3
    )
    
    print("New Translation", newParams[3])
    print("New Rotation", newParams[:3])
    print()
    return CalibReqResponse(translation=newParams[3], euler_angles=newParams[:3])


if __name__ == "__main__":
    rospy.init_node('calibration')
    autocalib_enabled = rospy.get_param("/AS/P/ccat/matcher/common/autocalib", False)
    if (not autocalib_enabled):
        rospy.signal_shutdown()
    
    s = rospy.Service("/AS/P/ccat/calibration", CalibReq, calib)
    rospy.loginfo("[ccat/calibration] Ready to calibrate.")
    rospy.spin()
