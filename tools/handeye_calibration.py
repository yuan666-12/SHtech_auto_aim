import os
import cv2
import numpy as np
import glob

def eulerAnglesToRotationMatrix(theta_deg):
    yaw, pitch, roll = np.radians(theta_deg)  # Convert degrees to radians

    R_x = np.array([[1, 0, 0],
                    [0, np.cos(pitch), -np.sin(pitch)],
                    [0, np.sin(pitch), np.cos(pitch)]])
    R_y = np.array([[np.cos(roll), 0, np.sin(roll)],
                    [0, 1, 0],
                    [-np.sin(roll), 0, np.cos(roll)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    # Match common/datatype.hpp: yaw(Z), pitch(X), roll(Y).
    # OpenCV expects gripper(IMU)-to-base(world) rotations here.
    R = R_z @ R_x @ R_y
    return R

def load_gimbal_euler_angles(directory):
    """
    Load gimbal angles and return a dict: {identifier: (yaw, pitch, roll)}
    """
    euler_dict = {}
    for txt_path in glob.glob(os.path.join(directory, 'euler_*.txt')):
        identifier = os.path.splitext(os.path.basename(txt_path))[0].split('_')[1]
        try:
            with open(txt_path, 'r') as f:
                line = f.readline().strip()
                if line:
                    yaw, pitch, roll = map(float, line.split())
                    euler_dict[identifier] = (yaw, pitch, roll)
                    # print(euler_dict[identifier])
        except Exception as e:
            print(f"Error reading {txt_path}: {e}")
    return euler_dict

def find_chessboard_corners(images, board_size, mtx, dist, euler_dict):
    """
    Detect chessboard and return ONLY matched poses with valid gimbal data.
    """
    square_size = 0.015  # Hero board square size: 15 mm
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_size

    # objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    # objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)

    rvecs_list = []
    tvecs_list = []
    R_gimbals = []

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for img_path in images:
        identifier = os.path.splitext(os.path.basename(img_path))[0].split('_')[1]
        if identifier not in euler_dict:
            print(f"Warning: No pose file for {img_path}")
            continue

        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Could not read {img_path}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, board_size, None)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            _, rvec, tvec = cv2.solvePnP(objp, corners2, mtx, dist)

            # Append only if both detection and pose exist
            rvecs_list.append(rvec)
            tvecs_list.append(tvec)
            R_gimbals.append(eulerAnglesToRotationMatrix(euler_dict[identifier]))
        else:
            print(f"Chessboard not found in {img_path}")

    return R_gimbals, rvecs_list, tvecs_list

if __name__ == "__main__":
    directory = "./hero_extrinsic_data_20260904_v2/"
    board_size = (11, 8)

    # Camera intrinsics (reshape to proper matrices)
    mtx = np.array([
        [1.4790807932250757e+03, 0.0, 7.3069968486430150e+02],
        [0.0, 1.4781439161714959e+03, 5.4818014607939051e+02],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)

    dist = np.array([-1.3372672103392352e-01, 2.1795172844506383e-01,
                     -4.5119642764941177e-04, 8.1427509035456589e-04,
                     -1.1184162108320281e-01], dtype=np.float64)

    # Load gimbal poses as dict
    euler_dict = load_gimbal_euler_angles(directory)

    # Get sorted image list
    images = sorted(glob.glob(os.path.join(directory, 'frame_*.png')))

    # Match and extract valid poses
    R_gimbals, rvecs, tvecs = find_chessboard_corners(images, board_size, mtx, dist, euler_dict)

    print(f"Valid pose pairs: {len(R_gimbals)}")

    if len(R_gimbals) < 3:
        print("Error: Need at least 3 valid pose pairs for hand-eye calibration!")
        exit(1)

    # Hand-eye calibration: Eye-to-hand (gimbal = "gripper", chessboard = "world")
    # R_gimbals: from world to gimbal
    # rvecs/tvecs: from world to camera (as returned by solvePnP)
    t_gimbals = np.zeros((len(R_gimbals), 3))  # assuming pure rotation (no translation)

    R_g2c, t_g2c = cv2.calibrateHandEye(R_gimbals, t_gimbals, rvecs, tvecs)

    print("Camera to IMU transformation (candidate R_c2i/T_c2i):")
    print("R_c2i:\n", R_g2c)
    print("T_c2i:\n", t_g2c)
