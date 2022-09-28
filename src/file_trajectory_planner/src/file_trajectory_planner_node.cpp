#include <string>
#include "rclcpp/rclcpp.hpp"

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

#include <file_trajectory_planner/file_trajectory_planner.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
using Point = geometry_msgs::msg::Point;
using ColorRGBA = std_msgs::msg::ColorRGBA;

using std::placeholders::_1;
using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;
using Marker = visualization_msgs::msg::Marker;
Marker create_points_marker(const std::string type_name, std::vector<TrajectoryPoint> &points,float z,float r, float g, float b, bool delete_marker,builtin_interfaces::msg::Time t);

class FileTrajectoryPlannerNode : public rclcpp::Node
{
public:
  FileTrajectoryPlannerNode()
  : Node("file_trajectory_planner"), count_(0)
  {
    // to change into test.txt
    const std::string file_name = declare_parameter("trajectory_file_name").get<std::string>();
   
     RCLCPP_INFO(rclcpp::get_logger("logger"), "file_name %s",file_name.c_str()); 
    const bool compute_velocity = declare_parameter("compute_velocity").get<bool>();
    declare_parameter("also_behind",false);
    // const bool also_behind = declare_parameter("also_behind",true).get<bool>();
    
    bool also_behind;
    this->get_parameter("also_behind", also_behind);
    file_trajextory_planner_.init_traj(file_name,compute_velocity,also_behind);

    //publishers:
    pub_trajectory_ = this->create_publisher<autoware_auto_msgs::msg::Trajectory>("trajectory", 10);
    pub_marker_ = this->create_publisher<Marker>("trajectory_marker", 10);
    //subscribers
    sub_kinematic_state_ = this->create_subscription<VehicleKinematicState>(
      "vehicle_state", 10, std::bind(&FileTrajectoryPlannerNode::on_state, this, _1));
  }

private:
  void on_state(const VehicleKinematicState::SharedPtr msg)
  {
    //RCLCPP_INFO(this->get_logger(), "on state");
    state_ = *msg;
    Trajectory traj = file_trajextory_planner_.get_trajectory(state_);
    traj.header = state_.header;
    pub_trajectory_->publish(traj);

    std::vector<TrajectoryPoint> traj_marker;
    for (size_t i = 0; i <traj.points.size();i++ )
    {
    traj_marker.push_back(traj.points[i]);
    }
    Marker marker=create_points_marker("recorder_trajecotory",traj_marker,0.2,0.445, 0.62, 0.8,false,state_.header.stamp);
    pub_marker_->publish(marker);
  }

  //publishers:
  rclcpp::Publisher<autoware_auto_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<Marker>::SharedPtr pub_marker_;
  //subscribers:
  rclcpp::Subscription<VehicleKinematicState>::SharedPtr sub_kinematic_state_;

  FileTrajectoryPlanner file_trajextory_planner_;
  VehicleKinematicState state_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FileTrajectoryPlannerNode>());
  rclcpp::shutdown();
  return 0;
}


Marker create_points_marker(const std::string type_name, std::vector<TrajectoryPoint> &points,float z,float r, float g, float b, bool delete_marker,builtin_interfaces::msg::Time t)
  {
    Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = t;
    marker.ns = type_name;
    marker.type = Marker::POINTS;
    if (delete_marker)
    {
      marker.action = Marker::DELETE;
    }
    else
    {
      marker.action = Marker::ADD;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      ColorRGBA c;
      c.a = 1.0;
      c.b = b;
      c.r = r;
      c.g = g;
      

      for(size_t i = 0;i<points.size(); i++){
        Point p;
        p.x = points[i].x;
        p.y = points[i].y;
        p.z = z;
        marker.points.push_back(p);
        marker.colors.push_back(c);
      }
    }

    return marker;
  }