#ifndef INFLATED_ROI_OCTREE_DISPLAY_H
#define INFLATED_ROI_OCTREE_DISPLAY_H

#include "octomap_rviz_plugins/occupancy_grid_display.h"
#include "octomap_vpp/InflatedRoiOcTree.h"

namespace octomap_vpp_rviz_plugin
{

class InflatedRoiOctreeDisplay : public octomap_rviz_plugin::OccupancyGridDisplay
{
Q_OBJECT
public:
  InflatedRoiOctreeDisplay();
protected:
  void incomingMessageCallback(const octomap_msgs::OctomapConstPtr& msg);
  void setVoxelColor(rviz::PointCloud::Point& newPoint, octomap_vpp::InflatedRoiOcTreeNode& node, double minZ, double maxZ, float maxRoiVal);
  ///Returns false, if the type_id (of the message) does not correspond to the template paramter
  ///of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);
};

}

#endif // INFLATED_ROI_OCTREE_DISPLAY_H
