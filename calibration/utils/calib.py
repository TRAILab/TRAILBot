import cv2
import numpy as np
import glob

# Checkerboard settings (inner corners)
CHECKERBOARD = (8, 6)
square_size = 0.120  # meters

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []
img_names = []

# Load images
images = glob.glob(
    "/home/trailbot/action4/20260227/outdoor_standard/select_intrinsic/*.jpg"
)

print(f"Found {len(images)} images.")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if found:
        corners2 = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        objpoints.append(objp)
        imgpoints.append(corners2)
        img_names.append(fname)

print(f"Valid detections: {len(objpoints)}")

# Calibration
ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\n=== Calibration Result ===")
print("RMS reprojection error:", ret)
print("Camera matrix K:\n", K)
print("Distortion coefficients:\n", D.ravel())

# Per-image reprojection error
total_error = 0.0

print("\nPer-image reprojection error:")
for i in range(len(objpoints)):
    imgpoints_proj, _ = cv2.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], K, D
    )
    err = cv2.norm(imgpoints[i], imgpoints_proj, cv2.NORM_L2) / len(imgpoints_proj)
    total_error += err
    print(f"{img_names[i]} : {err:.4f} px")

print(f"\nMean reprojection error: {total_error / len(objpoints):.4f} px")

total_sq_err = 0.0
total_points = 0

for i in range(len(objpoints)):
    proj, _ = cv2.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], K, D
    )

    diff = imgpoints[i].reshape(-1,2) - proj.reshape(-1,2)
    total_sq_err += np.sum(diff**2)
    total_points += diff.shape[0]

rms = np.sqrt(total_sq_err / total_points)
print("Global RMS reprojection error:", rms)
# -------------------------
# Distortion visualization
# -------------------------

for i, fname in enumerate(img_names):
    img = cv2.imread(fname)
    if img is None:
        continue

    # 原始检测到的角点
    corners = imgpoints[i]                  # (N,1,2)
    corners = corners.reshape(-1, 2)         # (N,2)

    # 重投影角点
    proj, _ = cv2.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], K, D
    )
    proj = proj.reshape(-1, 2)

    vis = img.copy()

    # 画检测角点（绿色）
    for p in corners:
        cv2.circle(
            vis,
            (int(round(p[0])), int(round(p[1]))),
            4,
            (0, 255, 0),
            -1,
        )

    # 画重投影角点（红色）
    for p in proj:
        cv2.circle(
            vis,
            (int(round(p[0])), int(round(p[1]))),
            6,
            (0, 0, 255),
            1,
        )

    cv2.putText(
        vis,
        f"{i} reproj err shown",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 0, 0),
        2,
    )

    cv2.imshow("Corners (Green) vs Reprojection (Red)", vis)
    key = cv2.waitKey(0)

    if key == 27:  # ESC 退出
        break

cv2.destroyAllWindows()