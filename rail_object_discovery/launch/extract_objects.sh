echo 'calling /discover_objects service with parameters tuned by rail'
rosservice call /discover_objects "constraints:
  object_min_sensor_range: 0.2
  object_max_sensor_range: 4.0
  object_min_spherical_radius: 0.01
  object_max_spherical_radius: 0.1
plane_slope_tolerance: 2.0"
