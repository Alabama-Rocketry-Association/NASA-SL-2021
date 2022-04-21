import cv2
import pickle

im=cv2.imread("FinalLaunch.jpg")
gr=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
d=cv2.SIFT_create()
kp, des=d.detectAndCompute(gr,None)

index = []

for point in range(len(kp)):
    temp = (kp[point].pt, kp[point].size, kp[point].angle, kp[point].response, kp[point].octave,
            kp[point].class_id)
    index.append(temp)
# for point in kp:
#     temp = (point.pt, point.size, point.angle, point.response, point.octave,
#         point.class_id)
#     index.append(temp)

# Dump the keypoints
f = open("keypoints.txt", "wb")
f.write(pickle.dumps(index))
f.close()
f = open("descriptors.txt", "wb")
f.write(pickle.dumps(des))
f.close()

