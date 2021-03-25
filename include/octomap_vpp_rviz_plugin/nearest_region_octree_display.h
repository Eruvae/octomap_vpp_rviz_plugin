#ifndef NEAREST_REGION_OCTREE_DISPLAY_H
#define NEAREST_REGION_OCTREE_DISPLAY_H

#include "octomap_rviz_plugins/occupancy_grid_display.h"
#include "octomap_vpp/NearestRegionOcTree.h"

namespace octomap_vpp_rviz_plugin
{

class NearestRegionOcTreeDisplay : public octomap_rviz_plugin::OccupancyGridDisplay
{
Q_OBJECT
public:
  NearestRegionOcTreeDisplay();
  virtual void update(float wall_dt, float ros_dt) override;

protected:
  void incomingMessageCallback(const octomap_msgs::OctomapConstPtr& msg);
  void setVoxelColor(rviz::PointCloud::Point& newPoint, octomap_vpp::NearestRegionOcTreeNode& node, double minZ, double maxZ, double maxDist);
  ///Returns false, if the type_id (of the message) does not correspond to the template paramter
  ///of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);

  void resubscribe();
};

}

#endif // NEAREST_REGION_OCTREE_DISPLAY_H
