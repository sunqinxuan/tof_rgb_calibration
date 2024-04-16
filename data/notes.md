#### second_round_back.bag

- /Preposition_NavigationCamera_left
- /Preposition_NavigationCamera_right
- /Preposition_ObstacleCamera_left
- /Preposition_ObstacleCamera_right
- /atti_esti_data
- /chassis_command
- /chassis_detail
- /chenxl_data
- /clock
- /curvature_odom_goal
- /ddem
- /front_lidar_data
- /g_cl
- /g_gl
- /imu_data
- /ir_image
- /joint_states
- /move_ctrl_data
- /multi_spectral0
- /multi_spectral2
- /multi_spectral3
- /multi_spectral4
- /multi_spectral5
- /plan_state
- /pos_esti_data
- /pq_data
- /publish_status
- /rosout
- /rosout_agg
- /up_tof
- /up_tof_confidence
- /up_tof_intensity
- /up_tof_range
- /up_tof_range_camera_info




| topic                               | type              | encoding | resolution | step         |
| ----------------------------------- | ----------------- | -------- | ---------- | ------------ |
| /Preposition_NavigationCamera_left  | sensor_msgs/Image | rgb8     | 2048x2048  | 6144(2048x3) |
| /Preposition_NavigationCamera_right | sensor_msgs/Image | rgb8     | 2048x2048  | 6144         |
| /up_tof_range                       | sensor_msgs/Image | mono16   | 480x640    | 1280(640x2)  |
| /up_tof_intensity                   | sensor_msgs/Image | mono16   | 480x640    | 1280         |



cv_bridge 

```cpp
mono8: CV_8UC1, grayscale image
mono16: CV_16UC1, 16-bit grayscale image
bgr8: CV_8UC3, color image with blue-green-red color order
rgb8: CV_8UC3, color image with red-green-blue color order
bgra8: CV_8UC4, BGR color image with an alpha channel
rgba8: CV_8UC4, RGB color image with an alpha channel
```