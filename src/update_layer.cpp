#include<lane_layer/update_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

GridLayer::GridLayer() {}

void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  points_sub_ = nh.subscribe("/people", 1, &GridLayer::stringMessage, this);
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}
void GridLayer::stringMessage(const string string_msg)
{
    points_msg_ = string_msg;
    string arr[8];
    stringstream ssin(points_msg_);
    int i=0;
    while (ssin.good() && i < 8){
        ssin >> arr[i];
        ++i;
    }
    line1[0][0] = stoi(arr[0]);
    line1[0][1] = stoi(arr[1]);
    line1[1][0] = stoi(arr[2]);
    line1[1][1] = stoi(arr[3]);
    line2[0][0] = stoi(arr[4]);
    line2[0][1] = stoi(arr[5]);
    line2[1][0] = stoi(arr[6]);
    line2[1][1] = stoi(arr[7]);


}


void GridLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  // double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  if(line1[0][0]==line1[1][0])
  {
    for(int i=line1[0][1];i<=line1[1][1];i++)
    {
      double mark_x = line1[0][0];
      double mark_y = i;
      unsigned int mx;
      unsigned int my;
      if(worldToMap(mark_x, mark_y, mx, my)){
        setCost(mx, my, LETHAL_OBSTACLE);
      }

      *min_x = std::min(*min_x, mark_x);
      *min_y = std::min(*min_y, mark_y);
      *max_x = std::max(*max_x, mark_x);
      *max_y = std::max(*max_y, mark_y);
    }
  }
  else
  {
    double slope = (line1[0][1] - line1[1][1]) / (1.0 * (line1[0][0] - line1[1][0]);
    for(int i = line1[0][0];i<=line1[1][0];i++)
    {
      double mark_x = i;
      double mark_y = slope * (i - line1[0][0]) + line1[0][1];
      unsigned int mx;
      unsigned int my;
      if(worldToMap(mark_x, mark_y, mx, my)){
        setCost(mx, my, LETHAL_OBSTACLE);
      }

      *min_x = std::min(*min_x, mark_x);
      *min_y = std::min(*min_y, mark_y);
      *max_x = std::max(*max_x, mark_x);
      *max_y = std::max(*max_y, mark_y);
    }
  }
  if(line2[0][0]==line2[1][0])
  {
    for(int i=line2[0][1];i<=line2[1][1];i++)
    {
      double mark_x = line2[0][0];
      double mark_y = i;
      unsigned int mx;
      unsigned int my;
      if(worldToMap(mark_x, mark_y, mx, my)){
        setCost(mx, my, LETHAL_OBSTACLE);
      }

      *min_x = std::min(*min_x, mark_x);
      *min_y = std::min(*min_y, mark_y);
      *max_x = std::max(*max_x, mark_x);
      *max_y = std::max(*max_y, mark_y);
    }
  }
  else
  {
    double slope = (line2[0][1] - line2[1][1]) / (1.0 * (line2[0][0] - line2[1][0]);
    for(int i = line2[0][0];i<=line2[1][0];i++)
    {
      double mark_x = i;
      double mark_y = slope * (i - line2[0][0]) + line2[0][1];
      unsigned int mx;
      unsigned int my;
      if(worldToMap(mark_x, mark_y, mx, my)){
        setCost(mx, my, LETHAL_OBSTACLE);
      }

      *min_x = std::min(*min_x, mark_x);
      *min_y = std::min(*min_y, mark_y);
      *max_x = std::max(*max_x, mark_x);
      *max_y = std::max(*max_y, mark_y);
    }
  }



}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

} // end namespace