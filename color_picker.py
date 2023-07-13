from skimage import io
import cv2


def nothing(x):
    pass


cv2.namedWindow('Trackbars')

cv2.createTrackbar('H', 'Trackbars', 0, 180, nothing)
cv2.createTrackbar('S', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('V', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('mH', 'Trackbars', 0, 180, nothing)
cv2.createTrackbar('mS', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('mV', 'Trackbars', 0, 255, nothing)

cv2.setTrackbarPos('mH', 'Trackbars', 180)
cv2.setTrackbarPos('mS', 'Trackbars', 255)
cv2.setTrackbarPos('mV', 'Trackbars', 255)

# url = input()
# frame = io.imread(url)

frame = cv2.imread('data/snapshot.jpeg')
rainbow = cv2.imread('data/rainbow2.jpg')

while True:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    rainbow_hsv = cv2.cvtColor(rainbow, cv2.COLOR_BGR2HSV)

    H = cv2.getTrackbarPos('H', 'Trackbars')
    S = cv2.getTrackbarPos('S', 'Trackbars')
    V = cv2.getTrackbarPos('V', 'Trackbars')
    mH = cv2.getTrackbarPos('mH', 'Trackbars')
    mS = cv2.getTrackbarPos('mS', 'Trackbars')
    mV = cv2.getTrackbarPos('mV', 'Trackbars')

    mask = cv2.inRange(hsv, (H, S, V), (mH, mS, mV))
    rainbow_mask = cv2.inRange(rainbow_hsv, (H, S, V), (mH, mS, mV))

    show = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('Trackbars', show)

    rainbow_show = cv2.bitwise_and(rainbow, rainbow, mask=rainbow_mask)
    cv2.imshow('Rainbow', rainbow_show)

    if cv2.waitKey(1) == ord('q'):
        break
