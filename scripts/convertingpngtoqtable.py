import cv2

# Load your RViz screenshot
img = cv2.imread('/home/ubuntu/com760cw2_group6/src/com760cw2_group6/map_for_r1.png')

# 1. Convert to Grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 2. Apply Threshold (Turning everything into pure Black or pure White)
# This makes the Path White and everything else Black
_, bw_image = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# 3. Save the Black and White version
cv2.imwrite('path_black_and_white.png', bw_image)
print("Black and White path image saved!")
