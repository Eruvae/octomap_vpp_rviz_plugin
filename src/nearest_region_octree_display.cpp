#include "octomap_vpp_rviz_plugin/nearest_region_octree_display.h"
#include "octomap_vpp_rviz_plugin/glasbey.h"

#include "rviz/properties/enum_property.h"
#include "rviz/properties/status_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"

#include "octomap_msgs/conversions.h"

namespace octomap_vpp_rviz_plugin
{

static const std::size_t max_octree_depth_ = sizeof(unsigned short) * 8;

enum OctreeVoxelRenderMode
{
  OCTOMAP_INFLATED_REGIONS,
  OCTOMAP_CORE_REGIONS
};

enum OctreeVoxelColorMode
{
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_DISCRETE_COLOR
};

using rviz::StatusProperty;
using rviz::PointCloud;

NearestRegionOcTreeDisplay::NearestRegionOcTreeDisplay() : octomap_rviz_plugin::OccupancyGridDisplay()
{
  int pInd = octree_render_property_->rowNumberInParent();
  this->removeChildren(pInd, 1);
  octree_render_property_ = new rviz::EnumProperty( "Voxel Rendering", "With inflated regions",
                                                    "Select voxel type.",
                                                     0,
                                                     SLOT( updateOctreeRenderMode() ),
                                                     this);

  addChild(octree_render_property_, pInd);

  octree_render_property_->addOption("With inflated regions", OCTOMAP_INFLATED_REGIONS);
  octree_render_property_->addOption("Only core regions", OCTOMAP_CORE_REGIONS);

  pInd = octree_coloring_property_->rowNumberInParent();
  this->removeChildren(pInd, 1);
  octree_coloring_property_ = new rviz::EnumProperty( "Voxel Coloring", "Discrete",
                                                "Select voxel coloring mode",
                                                0,
                                                SLOT( updateOctreeColorMode() ),
                                                this);
  addChild(octree_coloring_property_, pInd);

  octree_coloring_property_->addOption( "Discrete", OCTOMAP_DISCRETE_COLOR );
  octree_coloring_property_->addOption( "Z-Axis",  OCTOMAP_Z_AXIS_COLOR );
}

void NearestRegionOcTreeDisplay::incomingMessageCallback(const octomap_msgs::OctomapConstPtr& msg)
{
  ++messages_received_;
  setStatus(StatusProperty::Ok, "Messages", QString::number(messages_received_) + " octomap messages received");
  setStatusStd(StatusProperty::Ok, "Type", msg->id.c_str());
  if(!checkType(msg->id)){
    setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
    return;
  }

  ROS_DEBUG("Received OctomapBinary message (size: %d bytes)", (int)msg->data.size());

  header_ = msg->header;
  if (!updateFromTF()) {
      std::stringstream ss;
      ss << "Failed to transform from frame [" << header_.frame_id << "] to frame ["
          << context_->getFrameManager()->getFixedFrame() << "]";
      setStatusStd(StatusProperty::Error, "Message", ss.str());
      return;
  }

  // creating octree
  octomap_vpp::NearestRegionOcTree* octomap = NULL;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree){
    octomap = dynamic_cast<octomap_vpp::NearestRegionOcTree*>(tree);
    if(!octomap){
      setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
      return;
    }
  }
  else
  {
    setStatusStd(StatusProperty::Error, "Message", "Failed to deserialize octree message.");
    return;
  }


  tree_depth_property_->setMax(octomap->getTreeDepth());

  // get dimensions of octree
  double minX, minY, minZ, maxX, maxY, maxZ;
  octomap->getMetricMin(minX, minY, minZ);
  octomap->getMetricMax(maxX, maxY, maxZ);
  double max_dist = octomap->computeMaxDist();

  // reset rviz pointcloud classes
  for (std::size_t i = 0; i < max_octree_depth_; ++i)
  {
    point_buf_[i].clear();
    box_size_[i] = octomap->getNodeSize(i + 1);
  }

  size_t pointCount = 0;
  {
    // traverse all leafs in the tree:
    unsigned int treeDepth = std::min<unsigned int>(tree_depth_property_->getInt(), octomap->getTreeDepth());
    double maxHeight = std::min<double>(max_height_property_->getFloat(), maxZ);
    double minHeight = std::max<double>(min_height_property_->getFloat(), minZ);
    int stepSize = 1 << (octomap->getTreeDepth() - treeDepth); // for pruning of occluded voxels
    for (octomap_vpp::NearestRegionOcTree::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
    {
        bool render_condition = it.getZ() <= maxHeight && it.getZ() >= minHeight;
        if (render_condition)
        {
          OctreeVoxelRenderMode octree_render_mode = static_cast<OctreeVoxelRenderMode>(octree_render_property_->getOptionInt());
          bool display_voxel = true;
          if (octree_render_mode == OCTOMAP_CORE_REGIONS && it->getValue().distance != 0)
            display_voxel = false;

          if (display_voxel)
          {
            PointCloud::Point newPoint;

            newPoint.position.x = it.getX();
            newPoint.position.y = it.getY();
            newPoint.position.z = it.getZ();

            setVoxelColor(newPoint, *it, minZ, maxZ, max_dist);
            // push to point vectors
            unsigned int depth = it.getDepth();
            point_buf_[depth - 1].push_back(newPoint);

            ++pointCount;
          }
        }

    }
  }

  if (pointCount)
  {
    boost::mutex::scoped_lock lock(mutex_);

    new_points_received_ = true;

    for (size_t i = 0; i < max_octree_depth_; ++i)
      new_points_[i].swap(point_buf_[i]);

  }
  delete octomap;
}

void NearestRegionOcTreeDisplay::setVoxelColor(rviz::PointCloud::Point& newPoint, octomap_vpp::NearestRegionOcTreeNode &node, double minZ, double maxZ, double maxDist)
{
    OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt());
    octomap_vpp::RegionInfo regInf = node.getValue();
    //const double MIN_ALPHA = 0.01, MAX_ALPHA = 0.1;
    double alpha = regInf.distance == 0 ? 1.0 : 0.01;//MIN_ALPHA + (maxDist - regInf.distance) / maxDist * (MAX_ALPHA - MIN_ALPHA);
    switch (octree_color_mode)
    {
      case OCTOMAP_Z_AXIS_COLOR:
        setColor(newPoint.position.z, minZ, maxZ, color_factor_, newPoint);
        break;
      case OCTOMAP_DISCRETE_COLOR:
        newPoint.setColor(glasbey[regInf.nearestRegionId % 256][0] / 255.f,
                          glasbey[regInf.nearestRegionId % 256][1] / 255.f,
                          glasbey[regInf.nearestRegionId % 256][2] / 255.f,
                          alpha);
        break;
      default:
        break;
    }
}

void NearestRegionOcTreeDisplay::update(float wall_dt, float ros_dt)
{
  if (new_points_received_)
  {
    boost::mutex::scoped_lock lock(mutex_);

    for (size_t i = 0; i < max_octree_depth_; ++i)
    {
      double size = box_size_[i];

      cloud_[i]->clear();
      cloud_[i]->setDimensions(size, size, size);

      cloud_[i]->addPoints(&new_points_[i].front(), new_points_[i].size());
      new_points_[i].clear();
      cloud_[i]->setAlpha(alpha_property_->getFloat(), true);
    }
    new_points_received_ = false;
  }
  updateFromTF();
}

bool NearestRegionOcTreeDisplay::checkType(std::string type_id)
{
  if(type_id == "NearestRegionOcTree") return true;
  else return false;
}

void NearestRegionOcTreeDisplay::resubscribe()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( octomap_vpp_rviz_plugin::NearestRegionOcTreeDisplay, rviz::Display)
