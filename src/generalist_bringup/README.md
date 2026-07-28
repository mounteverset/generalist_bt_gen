# generalist_bringup

Holds project-wide configuration bundles and launch files. Use the `generalist_bringup.launch.py` entrypoint to start the default behavior tree executor with the shared parameter file located in `config/bt_executor_params.yaml`.

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
