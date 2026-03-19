import cv2
import numpy as np
import glob
import os

# ===============================
# USER CONFIG
# ===============================
IMG_GLOB = "/home/trailbot/action4/20260227/outdoor_ultra/select_intrinsic/*.jpg"
CROP_RATIO = 0.7            # 0.9 / 0.8 / 0.7 逐级尝试
CHECKERBOARD = (8, 6)       # inner corners (cols, rows)
SQUARE_SIZE = 0.120         # meters

# ===============================
# Helper: center crop
# ===============================
def center_crop(img, crop_ratio):
    h, w = img.shape[:2]
    new_h = int(h * crop_ratio)
    new_w = int(w * crop_ratio)
    y0 = (h - new_h) // 2
    x0 = (w - new_w) // 2
    return img[y0:y0 + new_h, x0:x0 + new_w]

# ===============================
# Prepare object points
# ===============================
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []
img_names = []

# ===============================
# Load images & detect corners
# ===============================
images = sorted(glob.glob(IMG_GLOB))
print(f"Found {len(images)} images")

for fname in images:
    img_raw = cv2.imread(fname)
    if img_raw is None:
        continue

    img = center_crop(img_raw, CROP_RATIO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH +
        cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if not found:
        continue

    corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )

    objpoints.append(objp)
    imgpoints.append(corners)
    img_names.append(fname)

print(f"Valid detections after crop={CROP_RATIO}: {len(objpoints)}")
assert len(objpoints) > 0

img_h, img_w = gray.shape[:2]

# ===============================
# Restricted pinhole calibration
# ===============================
flags = (
    cv2.CALIB_ZERO_TANGENT_DIST |  # p1 = p2 = 0
    cv2.CALIB_FIX_K3 |
    cv2.CALIB_FIX_K4 |
    cv2.CALIB_FIX_K5
)

ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    (img_w, img_h),
    None,
    None,
    flags=flags
)

print("\n=== Calibration Result (CROPPED ULTRA) ===")
print("Crop ratio:", CROP_RATIO)
print("RMS reprojection error:", ret)
print("Camera matrix K:\n", K)
print("Distortion D:", D.ravel())

# ===============================
# Global RMS reprojection error
# ===============================
total_sq_err = 0.0
total_points = 0

print("\nPer-image reprojection error:")
for i in range(len(objpoints)):
    proj, _ = cv2.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], K, D
    )

    diff = imgpoints[i].reshape(-1, 2) - proj.reshape(-1, 2)
    err = np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))

    total_sq_err += np.sum(diff ** 2)
    total_points += diff.shape[0]

    print(f"{os.path.basename(img_names[i])}: {err:.3f} px")

global_rms = np.sqrt(total_sq_err / total_points)
print(f"\nGlobal RMS reprojection error: {global_rms:.3f} px")

# ===============================
# Visualization: detected vs reprojection
# ===============================
for i, fname in enumerate(img_names):
    img_raw = cv2.imread(fname)
    img = center_crop(img_raw, CROP_RATIO)
    vis = img.copy()

    corners = imgpoints[i].reshape(-1, 2)
    proj, _ = cv2.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], K, D
    )
    proj = proj.reshape(-1, 2)

    for p in corners:
        cv2.circle(vis, (int(p[0]), int(p[1])), 4, (0, 255, 0), -1)

    for p in proj:
        cv2.circle(vis, (int(p[0]), int(p[1])), 6, (0, 0, 255), 1)

    cv2.imshow("Green: detected | Red: reprojected (cropped)", vis)
    if cv2.waitKey(0) == 27:
        break

cv2.destroyAllWindows()

# ===============================
# Undistort ALL images (scaled view)
# ===============================
def resize_to_screen(img, max_w=1600, max_h=900):
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    return cv2.resize(img, (int(w * scale), int(h * scale)))

for fname in img_names:
    img_raw = cv2.imread(fname)
    img = center_crop(img_raw, CROP_RATIO)
    undist = cv2.undistort(img, K, D)

    vis = np.hstack((img, undist))
    vis = resize_to_screen(vis)

    cv2.imshow("Left: Cropped | Right: Undistorted", vis)
    if cv2.waitKey(0) == 27:
        break

cv2.destroyAllWindows()