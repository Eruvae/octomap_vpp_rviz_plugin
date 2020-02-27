#ifndef COUNTINGOCTREEDISPLAY_H
#define COUNTINGOCTREEDISPLAY_H

#include "octomap_rviz_plugins/occupancy_grid_display.h"
#include "octomap_vpp/CountingOcTree.h"

namespace octomap_vpp_rviz_plugin
{

class CountingOcTreeDisplay : public octomap_rviz_plugin::OccupancyGridDisplay
{
Q_OBJECT
public:
  CountingOcTreeDisplay();

private Q_SLOTS:
  void updateMinCount();
  void updateMinColor();
  void updateMaxColor();

protected:
  void incomingMessageCallback(const octomap_msgs::OctomapConstPtr& msg);
  void setVoxelColor(rviz::PointCloud::Point& newPoint, octomap_vpp::CountingOcTreeNode& node, double minZ, double maxZ);
  ///Returns false, if the type_id (of the message) does not correspond to the template paramter
  ///of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);

  void resubscribe();

  rviz::IntProperty *min_count_property_;
  rviz::IntProperty *min_color_property_;
  rviz::IntProperty *max_color_property_;
};

}

#endif // COUNTINGOCTREEDISPLAY_H
