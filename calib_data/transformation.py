import pathlib
import yaml
import numpy as np
import matplotlib.pyplot as plt
import pytransform3d.camera as pc
import pytransform3d.transformations as pt
import cv2

np.set_printoptions(suppress=True)
pathLeft = str(pathlib.Path(__file__).parent.resolve()) + "/stereo_left.yaml"
pathRight = str(pathlib.Path(__file__).parent.resolve()) + "/stereo_right.yaml"


def load_param(path):
    with open(path, "r") as f:
        param = yaml.load(f, yaml.FullLoader)

    height = param["image_height"]
    width = param["image_width"]
    distortion_model = param["distortion_model"]
    d = param["distortion_coefficients"]["data"]
    k = param["camera_matrix"]["data"]
    r = param["rectification_matrix"]["data"]
    p = param["projection_matrix"]["data"]
    return np.array([width, height]), np.array(k).reshape(3, 3), np.array(r).reshape(3, 3), np.array(p).reshape(3, 4)


cLsensorsize, cLK, cLR, cLP = load_param(pathLeft)
print(f"> cLsensorsize: {cLsensorsize}")
print(f"> cLK: {cLK}")
print(f"> cLR: {cLR}")
print(f"> cLP: {cLP}")
cRsensorsize, cRK, cRR, cRP = load_param(pathRight)
print(f"> cRsensorsize: {cRsensorsize}")
print(f"> cRK: {cRK}")
print(f"> cRR: {cRR}")
print(f"> cRP: {cRP}")


virtual_image_distance = 1

# origin
ax = pt.plot_transform()

cam1 = np.eye(4)
cam1[0:3, 0:3] = cLR
print(f"> cam1: {cam1}")


cam2 = np.eye(4)
cam2[0:3, 0:3] = cRR
cam2[0, 3] = cRP[0, 3] / -cRP[0, 0]
print(f"> cam2: {cam2}")

pt.plot_transform(ax, cam1, name="camera_1_main")
pt.plot_transform(ax, cam2, name="camera_2")
pc.plot_camera(ax, cam2world=cam1, M=cLK, sensor_size=cLsensorsize, virtual_image_distance=virtual_image_distance)
pc.plot_camera(ax, cam2world=cam2, M=cRK, sensor_size=cRsensorsize, virtual_image_distance=virtual_image_distance)
plt.show()
