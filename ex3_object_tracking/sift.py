import cv2
from matplotlib import pyplot as plt

SHOW_IMAGE = False
USE_SIFT = True
lowe_ratio = 0.4

img = cv2.imread('track.JPG')
template = cv2.imread('tag2.JPG',0)

# convert BRG to RGB then convert to grayscale
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

if SHOW_IMAGE:
    plt.imshow(img)
    plt.show()

if USE_SIFT:
    # code borrowed from https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html
    # Initiate SIFT detector
    # use pip install opencv-python==4.5.5.64 if SIFT isn't part of cv2
    sift = cv2.SIFT_create()
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img,None)
    kp2, des2 = sift.detectAndCompute(template,None)
    # FLANN parameters
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)   # or pass empty dictionary
    flann = cv2.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    # Need to draw only good matches, so create a mask
    matchesMask = [[0,0] for i in range(len(matches))]
    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
        if m.distance < lowe_ratio * n.distance:
            matchesMask[i]=[1,0]
            
    draw_params = dict(matchColor = (0,255,0),
                       singlePointColor = (255,0,0),
                       matchesMask = matchesMask,
                       flags = 2)
    img3 = cv2.drawMatchesKnn(img,kp1,template,kp2,matches,None,**draw_params)
    plt.imshow(img3,),plt.show()