# code adapted from https://pypi.org/project/pupil-apriltags/
# and https://pyimagesearch.com/2020/11/02/apriltag-with-python/

import cv2
from matplotlib import pyplot as plt
from pupil_apriltags import Detector

img = cv2.imread('track.JPG')
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

tags = at_detector.detect(img_gray, estimate_tag_pose=False, camera_params=None, tag_size=None)
print("number of tags detected:", len(tags))
# loop over the AprilTag detection results
for tag in tags:
    print("tag id:", tag.tag_id)
	# extract the bounding box (x, y)-coordinates for the AprilTag
	# and convert each of the (x, y)-coordinate pairs to integers
    (ptA, ptB, ptC, ptD) = tag.corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))
	# draw the bounding box of the AprilTag detection
    cv2.line(img, ptA, ptB, (0, 255, 0), 2)
    cv2.line(img, ptB, ptC, (0, 255, 0), 2)
    cv2.line(img, ptC, ptD, (0, 255, 0), 2)
    cv2.line(img, ptD, ptA, (0, 255, 0), 2)
	# draw the center (x, y)-coordinates of the AprilTag
    (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
    cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
	# draw the tag family on the image
    tagFamily = tag.tag_family.decode("utf-8")
    cv2.putText(img, tagFamily, (ptA[0], ptA[1] - 15),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    print("[INFO] tag family: {}".format(tagFamily))
# show the output image after AprilTag detection
    
plt.imshow(img)
plt.show()