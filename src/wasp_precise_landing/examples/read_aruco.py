import cv2
import numpy as np
import cv2.aruco as aruco

# https://fodi.github.io/arucosheetgen/
camera_matrix = np.array([
    [1.019099074177694320e+03,0.000000000000000000e+00,6.557727729771451095e+02],
    [0.000000000000000000e+00,1.011927236550148677e+03,3.816077913964442700e+02],
    [0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]
])

distCoeffs = np.array([2.576784605153304430e-01,-1.300640184051879311e+00,-4.285777480424158084e-03,-2.507657388926626523e-03,2.307018624520866812e+00])

frame = cv2.imread("/home/user/wasp_ws/src/wasp_precise_landing/data/test_0_shift.png")
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
parameters = aruco.DetectorParameters_create()  # Marker detection parameters# lists of ids and the corners beloning to each id
corners, ids, rejected_img_points = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
# print(corners)
if np.all(ids is not None):  # If there are markers found by detector
    for i in range(0, len(ids)):  # Iterate in markers
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, camera_matrix, distCoeffs)
        (rvec - tvec).any()  # get rid of that nasty numpy value array error
        print(rvec, tvec, markerPoints)
        aruco.drawAxis(frame, camera_matrix, distCoeffs, rvec, tvec, 0.01)  # Draw Axis
cv2.imshow("debug", frame)
cv2.waitKey(0)

cv2.destroyAllWindows()


