#include "octomap_vpp_rviz_plugin/counting_octree_display.h"
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
  OCTOMAP_ALL_VOXELS,
  OCTOMAP_OUTER_VOXELS
};

enum OctreeVoxelColorMode
{
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_COUNT_COLOR,
  OCTOMAP_DISCRETE_COLOR
};

using rviz::StatusProperty;
using rviz::PointCloud;

CountingOcTreeDisplay::CountingOcTreeDisplay() : octomap_rviz_plugin::OccupancyGridDisplay()
{
  int pInd = octree_render_property_->rowNumberInParent();
  this->removeChildren(pInd, 1);
  octree_render_property_ = new rviz::EnumProperty( "Voxel Rendering", "Outer Voxels",
                                                    "Select voxel type.",
                                                     0,
                                                     SLOT( updateOctreeRenderMode() ),
                                                     this);

  addChild(octree_render_property_, pInd);

  octree_render_property_->addOption("All Voxels", OCTOMAP_ALL_VOXELS);
  octree_render_property_->addOption("Outer Voxels", OCTOMAP_OUTER_VOXELS);

  pInd = octree_coloring_property_->rowNumberInParent();
  this->removeChildren(pInd, 1);
  octree_coloring_property_ = new rviz::EnumProperty( "Voxel Coloring", "Count",
                                                "Select voxel coloring mode",
                                                0,
                                                SLOT( updateOctreeColorMode() ),
                                                this);
  addChild(octree_coloring_property_, pInd);

  octree_coloring_property_->addOption( "Count",  OCTOMAP_COUNT_COLOR );
  octree_coloring_property_->addOption( "Discrete", OCTOMAP_DISCRETE_COLOR );
  octree_coloring_property_->addOption( "Z-Axis",  OCTOMAP_Z_AXIS_COLOR );

  min_count_property_ = new rviz::IntProperty("Min count", 0,
                                              "Select minimum count to display (0 = unlimited)",
                                              this,
                                              SLOT( updateMinCount() ),
                                              this);

  min_count_property_->setMin(0);

  min_color_property_ = new rviz::IntProperty("Min color", 0,
                                              "Select minimum color count",
                                              this,
                                              SLOT( updateMinColor() ),
                                              this);

  min_color_property_->setMin(0);
  min_color_property_->setMax(299);

  max_color_property_ = new rviz::IntProperty("Max color", 300,
                                              "Select maximum color count",
                                              this,
                                              SLOT( updateMaxColor() ),
                                              this);

  max_color_property_->setMin(1);
}

void CountingOcTreeDisplay::incomingMessageCallback(const octomap_msgs::OctomapConstPtr& msg)
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
  octomap_vpp::CountingOcTree* octomap = NULL;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree){
    octomap = dynamic_cast<octomap_vpp::CountingOcTree*>(tree);
    if(!octomap){
      setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
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
    for (octomap_vpp::CountingOcTree::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
    {
        int minCount = min_count_property_->getInt();
        bool render_condition = it.getZ() <= maxHeight && it.getZ() >= minHeight && it->getCount() >= minCount;
        if (render_condition)
        {
          OctreeVoxelRenderMode octree_render_mode = static_cast<OctreeVoxelRenderMode>(octree_render_property_->getOptionInt());
          bool render_all = (octree_render_mode == OCTOMAP_ALL_VOXELS);
          bool display_voxel = render_all;

          if (!render_all)
          {
            // check if current voxel has neighbors on all sides -> no need to be displayed
            bool allNeighborsFound = true;

            octomap::OcTreeKey key;
            octomap::OcTreeKey nKey = it.getKey();

            // determine indices of potentially neighboring voxels for depths < maximum tree depth
            // +/-1 at maximum depth, +2^(depth_difference-1) and -2^(depth_difference-1)-1 on other depths
            int diffBase = (it.getDepth() < octomap->getTreeDepth()) ? 1 << (octomap->getTreeDepth() - it.getDepth() - 1) : 1;
            int diff[2] = {-((it.getDepth() == octomap->getTreeDepth()) ? diffBase : diffBase + 1), diffBase};

            // cells with adjacent faces can occlude a voxel, iterate over the cases x,y,z (idxCase) and +/- (diff)
            for (unsigned int idxCase = 0; idxCase < 3; ++idxCase)
            {
              int idx_0 = idxCase % 3;
              int idx_1 = (idxCase + 1) % 3;
              int idx_2 = (idxCase + 2) % 3;

              for (int i = 0; allNeighborsFound && i < 2; ++i)
              {
                key[idx_0] = nKey[idx_0] + diff[i];
                // if rendering is restricted to treeDepth < maximum tree depth inner nodes with distance stepSize can already occlude a voxel
                for (key[idx_1] = nKey[idx_1] + diff[0] + 1; allNeighborsFound && key[idx_1] < nKey[idx_1] + diff[1]; key[idx_1] += stepSize)
                {
                  for (key[idx_2] = nKey[idx_2] + diff[0] + 1; allNeighborsFound && key[idx_2] < nKey[idx_2] + diff[1]; key[idx_2] += stepSize)
                  {
                    octomap_vpp::CountingOcTreeNode* node = octomap->search(key, treeDepth);

                    if (!(node && node->getCount() >= minCount))
                    {
                      // we do not have a neighbor => break!
                      allNeighborsFound = false;
                    }
                  }
                }
              }
            }

            display_voxel |= !allNeighborsFound;
          }

          if (display_voxel)
          {
            PointCloud::Point newPoint;

            newPoint.position.x = it.getX();
            newPoint.position.y = it.getY();
            newPoint.position.z = it.getZ();

            setVoxelColor(newPoint, *it, minZ, maxZ);
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

void CountingOcTreeDisplay::setVoxelColor(rviz::PointCloud::Point& newPoint, octomap_vpp::CountingOcTreeNode &node, double minZ, double maxZ)
{
    OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt());
    int minColor = min_color_property_->getInt();
    int maxColor = max_color_property_->getInt();
    unsigned int count = node.getCount();
    float reachability = (float)((int)count - minColor) / (float)(maxColor - minColor);
    reachability = std::max(0.f, std::min(1.f, reachability));
    switch (octree_color_mode)
    {
      case OCTOMAP_Z_AXIS_COLOR:
        setColor(newPoint.position.z, minZ, maxZ, color_factor_, newPoint);
        break;
      case OCTOMAP_COUNT_COLOR:
        newPoint.setColor((1.0f-reachability), reachability, 0.0);
        break;
      case OCTOMAP_DISCRETE_COLOR:
        newPoint.setColor(glasbey[count % 256][0] / 255.f, glasbey[count % 256][1] / 255.f, glasbey[count % 256][2] / 255.f);
        break;
      default:
        break;
    }
}

bool CountingOcTreeDisplay::checkType(std::string type_id)
{
  if(type_id == "CountingOcTree") return true;
  else return false;
}

void CountingOcTreeDisplay::resubscribe()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void CountingOcTreeDisplay::updateMinCount()
{
  resubscribe();
}

void CountingOcTreeDisplay::updateMinColor()
{
  max_color_property_->setMin(min_color_property_->getInt() + 1);
  resubscribe();
}

void CountingOcTreeDisplay::updateMaxColor()
{
  min_color_property_->setMax(max_color_property_->getInt() - 1);
  resubscribe();
}

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( octomap_vpp_rviz_plugin::CountingOcTreeDisplay, rviz::Display)
