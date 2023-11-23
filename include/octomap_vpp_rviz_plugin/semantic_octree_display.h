#pragma once

#include "octomap_rviz_plugins/occupancy_grid_display.h"
#include "octomap_vpp/SemanticOcTree.h"
#include "rviz/properties/color_property.h"

namespace octomap_vpp_rviz_plugin
{

class SemanticOcTreeDisplay : public octomap_rviz_plugin::OccupancyGridDisplay
{
Q_OBJECT
public:
  SemanticOcTreeDisplay();
protected:
  void incomingMessageCallback(const octomap_msgs::OctomapConstPtr& msg);
  void setVoxelColor(rviz::PointCloud::Point& newPoint, octomap_vpp::SemanticOcTreeNode& node, double minZ, double maxZ);
  ///Returns false, if the type_id (of the message) does not correspond to the template paramter
  ///of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);

  rviz::ColorProperty* min_color_property_;
  rviz::ColorProperty* max_color_property_;
};

}
