# generalist_bringup

Holds project-wide configuration bundles and launch files. Use the `generalist_bringup.launch.py` entrypoint to start the default behavior tree executor with the shared parameter file located in `config/bt_executor_params.yaml`.

For BlueBoat, start MAVROS with the hardware UDP endpoint:

```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://@192.168.2.2:14600 \
  -p system_id:=255 -p component_id:=190 \
  -p tgt_system:=1 -p tgt_component:=1
```

Then select its installed capability profile:

```bash
ros2 launch generalist_bringup generalist_bringup.launch.py \
  system_description_file:=$(ros2 pkg prefix mission_reasoner)/share/mission_reasoner/config/system_description_blueboat.yaml \
  gps_fix_topic:=/mavros/global_position/raw/fix
```

`clearpath_a200_navigation_sim.launch.py` also starts the mock GPS publisher and,
by default, `navsat_transform_node`. This exposes `/fromLL`, which Nav2's
`FollowGPSWaypoints` action uses to convert `MoveToGPS` goals. Set
`enable_gps_navigation:=false` to disable the converter.

The geographic tree IDs are `gps_waypoint_navigation.xml` for a plain route and
`gps_temperature_logging.xml` when temperature logging is requested. Both
consume `gps_waypoints`; map-frame trees continue to consume `waypoints`.

The simulation setup assumes that odometry yaw is aligned with ENU at startup
and does not broadcast an additional UTM transform, so it does not replace or
compete with SLAM. It is suitable for exercising the GPS action interface, but
the `solar_farm` world is not a geographic model of Hollerner Lake; a real lake
route still requires a matching map/world and globally referenced localization.
