# Arducam Shield ROS2

ROS2 package extension for arducam shield camera.

## Usages

- Execute the udev rule in libfile folder.
- Modify the name of desired camera config in launch file based on file in config folder.


```bash
ros2 launch arducam_shield single_cam.launch.py
```

```bash
ros2 launch arducam_shield stereo_cam_720.launch.py
```

## Camera Calibration
Run camera calibration if needed and copy the result from tmp folder and paste in calib_data folder.
```bash
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 8x6 --square 0.024 right:=/camera_right/image_raw left:=/camera_left/image_raw right_camera:=/camera_right left_camera:=/camera_left
```

## Reference
* [Arducam Library](https://github.com/ArduCAM/ArduCAM_USB_Camera_Shield/tree/master)
* [Download Camera Config Here](https://github.com/ArduCAM/ArduCAM_USB_Camera_Shield/tree/master/Config)
* [Arducam Config Parser](https://github.com/ArduCAM/arducam_config_parser/tree/master)
* [Arducam Python Demo](https://github.com/ArduCAM/ArduCAM_USB_Camera_Shield_Python_Demo?tab=readme-ov-file)



