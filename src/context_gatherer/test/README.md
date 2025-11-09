# Context Gatherer Manual Verification

1. Launch the context gatherer node with `ros2 launch context_gatherer context_gatherer.launch.py`.
2. In a separate terminal run `ros2 topic list` to ensure camera, odometry, map, and geolocation topics are present (e.g., `/camera/image_raw`, `/odometry/global`, `/map`, `/gps/geo`).
3. Call the service `ros2 service call /get_context std_srvs/srv/Trigger {}` and confirm the response message contains JSON with `camera`, `odometry`, `map`, and `geolocation` sections.
4. (Optional) To run the automated launch test in an environment with DDS networking enabled, configure the workspace with `colcon build --cmake-args -DCONTEXT_GATHERER_ENABLE_LAUNCH_TEST=ON` and then execute `colcon test --packages-select context_gatherer`.
