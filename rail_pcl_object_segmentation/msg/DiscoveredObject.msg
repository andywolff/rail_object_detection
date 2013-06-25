# Represents an object that has been discovered in a point cloud, with its cloud of points, center position, color, and radius

# The points that lie within the discovered plane
sensor_msgs/PointCloud2 objectCloud

#the center of the pointcloud
geometry_msgs/Point center

#the average color of the pointCloud
std_msgs/ColorRGBA color

#the spherical radius of the pointCloud
std_msgs/Float64 radius
