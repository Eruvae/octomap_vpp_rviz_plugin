#ifndef WORKSPACEOCTREEDISPLAY_H
#define WORKSPACEOCTREEDISPLAY_H

#include "octomap_rviz_plugins/occupancy_grid_display.h"
#include "octomap_vpp/WorkspaceOcTree.h"

namespace octomap_vpp_rviz_plugin
{

class WorkspaceOcTreeDisplay : public octomap_rviz_plugin::OccupancyGridDisplay
{
Q_OBJECT
public:
  WorkspaceOcTreeDisplay();

private Q_SLOTS:
  void updateMinReachability();

protected:
  void incomingMessageCallback(const octomap_msgs::OctomapConstPtr& msg);
  void setVoxelColor(rviz::PointCloud::Point& newPoint, octomap_vpp::WorkspaceNode& node, double minZ, double maxZ);
  ///Returns false, if the type_id (of the message) does not correspond to the template paramter
  ///of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);

  void resubscribe();

  rviz::FloatProperty *min_reachability_property_;
};

}

#endif // WORKSPACEOCTREEDISPLAY_H
