echo 'calling /discover_objects service with parameters tuned by rail'
rosservice call /discover_objects "constraints:
  object_min_sensor_range: 0.4
  object_max_sensor_range: 3.0
  object_min_spherical_radius: 0.03
  object_max_spherical_radius: 0.7
plane_slope_tolerance: 1.0"
