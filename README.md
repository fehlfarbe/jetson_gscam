# jetson_gscam

Hardware accelerated image capturing on NVidia Jetson devices via GStreamer pipeline.
The pipeline supports raw and JPEG compressed capturing.

## Nodes

### camera_node.py

Publishes raw and compressed images and CameraInfo.

#### Published Topics

`image` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
    Raw RGB image

`image/compressed` ([sensor_msgs/CompressedImage](http://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html))
    Compressed Image (JPEG encoded)

`camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))
    CameraInfo

#### Parameters

* `~sensor` (int, default: 0)
    camera sensor id for nvarguscamerasrc
* `~width` (int, default 1920)
    image width
* `~height` (int, default 1080)
    image height
* `~fps` (int, default 30)
    frames per second
* `~flip` (int, default 0)
    From NVidia Accelerated GStreamer user guide:
    | Flip Method | Value |
    |-------------|-------|
    | no rotation | 0     |
    | CCW 90°     | 1     |
    | 180°        | 2     |
    | CW 90°      | 3     |
    | hflip       | 4     |
    | upper right diagonal flip | 5     |
    | vflip       | 6     |
    | upper left diagonal flip | 7     |
* `~camera_info_url` (String, default: null)
    URL to camera calibration file. e.g. `file:///home/robot/calib.yml`. File is loaded by camera_info_manager_py.
* `~name` (String, default: "camera")
    Camera name
