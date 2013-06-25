/*!
 * \file pcl_measurement.hpp
 * \brief Defines common point cloud measurement routines.
 * 
 * \author Paul Malmsten, WPI - pmalmsten@gmail.com
 * \date Sep 21, 2012
 */

#ifndef _PCL_MEASUREMENT_H
#define _PCL_MEASUREMENT_H

#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <cmath>

#include "rail_pcl_object_segmentation/common.hpp"

namespace rail
{
pcl::PointXYZRGB AveragePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  unsigned int total_points = cloud->size();
  double x=0,y=0,z=0,r=0,g=0,b=0;
  for(pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = cloud->begin(); it != cloud->end(); ++it) {
     pcl::PointXYZRGB p = *it;
     x+=p.x;
     y+=p.y;
     z+=p.z;
     r+=p.r;
     g+=p.g;
     b+=p.b;
  }
  pcl::PointXYZRGB p;
  p.x=x/total_points;
  p.y=y/total_points;
  p.z=z/total_points;
  p.r=r/total_points;
  p.g=g/total_points;
  p.b=b/total_points;
  return p;
}

/*!
 * \brief Averages together the points in a point cloud and returns the resulting point.
 * 
 * \param [in] cloud The point cloud to average.
 * \return The averaged point within the cloud.
 */
template<typename PointT>
  PointT AveragePointCloudToPoint(typename pcl::PointCloud<PointT>::Ptr cloud)
  {
    PointT centerPoint;

    // Sum all points together
    for (typename pcl::PointCloud<PointT>::iterator cloud_point = cloud->begin(); cloud_point != cloud->end();
        ++cloud_point)
    {
      centerPoint.x += (*cloud_point).x;
      centerPoint.y += (*cloud_point).y;
      centerPoint.z += (*cloud_point).z;
    }

    // Divide by point count
    centerPoint.x /= cloud->size();
    centerPoint.y /= cloud->size();
    centerPoint.z /= cloud->size();

    return centerPoint;
  }

/*!
 * \brief Determines the bounding radius of the given point cloud from the given point. 
 * 
 * This function computes the radius of a bounding sphere for all
 * of the points in the given cloud. The bounding sphere is centered
 * at the given center point, and its radius is the minimum radius
 * such that the sphere contains all of the points in the given
 * point cloud.
 * 
 * \param [in] cloud The point cloud measure.
 * \param [in] center The point at which the center of the bounding sphere should be located.
 * \return The radius of the bounding sphere.
 */
template<typename PointT>
  double ComputePointCloudBoundingRadiusFromPoint(typename pcl::PointCloud<PointT>::Ptr cloud, PointT center)
  {
    double boundingRadius = 0;

    for (typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end(); ++it)
    {
      PointT currentPoint = *it;

      // Measure distance
      double radius = sqrt(
          pow(currentPoint.x - center.x, 2) + pow(currentPoint.y - center.y, 2) + pow(currentPoint.z - center.z, 2));

      if (radius > boundingRadius)
      {
        boundingRadius = radius;
      }
    }

    return boundingRadius;
  }



/**
 * \brief Computes the average hue of a point cloud.
 *
 * The hue for a cloud falls within the circular range [0°, 360°)
 *
 * \param [in] cloud The cloud to measure.
 * \return The average hue of all of the points in the cloud; in the circular range 0 - 2pi.
 */
double ComputePointCloudAverageHue(pcl::PointCloud<pcl::PointXYZHSV>::ConstPtr cloud);

/**
 * \brief Computes the average hue of a point cloud.
 *
 * The given point cloud of XYZRGB points is converted to a cloud of XYZHSV before
 * analysis.
 *
 * The hue for a cloud falls within the circular range [0°, 360°)
 *
 * \param [in] cloud The cloud to measure.
 * \return The average hue of all of the points in the cloud; in the circular range 0 - 2pi
 */
double ComputePointCloudAverageHue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

#define CUP_MAX_RADIUS 0.15
#define CUP_MIN_RADIUS 0.01
/*!
 * \brief Determines whether the given point cloud is the approximate size of a cup.
 */
template<typename PointT>
  struct is_cup_size : public std::unary_function<typename pcl::PointCloud<PointT>::Ptr, bool>
  {
    bool operator()(const typename pcl::PointCloud<PointT>::Ptr cloud) const
    {
      PointT center = rail::AveragePointCloudToPoint<PointT>(cloud);
      double radius = rail::ComputePointCloudBoundingRadiusFromPoint<PointT>(cloud, center);
      return radius > CUP_MIN_RADIUS && radius < CUP_MAX_RADIUS;
    }
  };

/*!
 * \brief Determines whether the given point cloud close enough to the origin.
 */
template<typename PointT>
  struct is_closer_than : public std::unary_function<typename pcl::PointCloud<PointT>::Ptr, bool>
  {
    double max_distance;
    bool operator()(const typename pcl::PointCloud<PointT>::Ptr cloud) const
    {
      PointT center = rail::AveragePointCloudToPoint<PointT>(cloud);
      double distance = sqrt(pow(center.x, 2.0) + pow(center.y, 2.0) + pow(center.z, 2.0));
      return distance < max_distance;
    }
  };


/**
 * \brief Determines whether the given point cloud has an average hue within the given bounds.
 *
 * The boolean field invert_range determines how the min_hue and max_hue are perceived. By default,
 * with invert_range = false, the average hue of a point cloud must be greater than or equal to min_hue and
 * less than or equal to max_hue.
 *
 * If invert_range = true, this is inverted; the average hue of a point cloud must
 * be less than to min_hue or greater than to max_hue.
 *
 * The hue for a cloud falls within the circular range between 0 and 2pi (note that 2pi wraps around
 * to 0).
 */
struct is_correct_color : public std::unary_function<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, bool>
{
  double min_hue;
  double max_hue;
  bool invert_range;

  bool operator() (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const;
};

/*!
 * \brief Determines whether the given plane is level within a given tolerance.
 * 
 * Given a plane equation in the form ax + by + cz + d = 0 in a DiscoveredPlane
 * structure, this function evaluates whether it is level within a given
 * tolerance.
 * 
 * This is measured by taking the partial derivative of the plane
 * with respect to x and y. These become:
 * 
 * df/dx: a = 0
 * df/dy: b = 0
 * 
 * Intuitively, a defines the slope of the plane along the x axis,
 * and b defines the slope of the plane along the y axis.
 * 
 * If either of these exceeds a given tolerance, then it is declared
 * that the plane is not level, and false is returned. Otherwise,
 * true is returned.
 */
struct is_level : public std::unary_function<rail::DiscoveredPlanePtr, bool>
{
  double max_slope;

  bool operator()(const rail::DiscoveredPlanePtr plane) const;
};

/**
 * \brief Removes point clouds which are not level.
 * 
 * \param [inout] planes The vector of planes to filter in-place.
 * \param [in] max_slope The maximum slope along the x or y axis that
 *  a plane must not exceed.
 */
void FilterInclinedPlanes(std::vector<rail::DiscoveredPlanePtr>& planes, double max_slope);

/**
 * \brief Removes point clouds with a hue that does not fall within the correct range.
 *
 * \param [inout] pointClouds The vector of point clouds to filer in-place.
 * \param [in] min_hue The minimum hue value that a cloud may present.
 * \param [in] max_hue The maximum hue value that a cloud may present.
 * \param [in] invert_range When false, only clouds with an average hue in the specified range are returned. If false, only
 *  only clouds with an average hue outside of the specified range are returned.
 */
void FilterWrongColor(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& pointClouds, double min_hue, double max_hue, bool invert_range);

}

#endif
