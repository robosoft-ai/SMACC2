#pragma once
#include <bond/msg/status.hpp>
#include <smacc2/client_behaviors/cb_wait_topic_message.hpp>
#include <smacc2/smacc.hpp>

using namespace std::chrono_literals;

namespace sm_aws_warehouse_navigation
{
// STATE DECLARATION
struct StAcquireSensors : smacc2::SmaccState<StAcquireSensors, SmAwsWarehouseNavigation>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbWaitNav2Nodes, OrNavigation>, StInitialNavigateForward, SUCCESS> >
    reactions;

  cl_move_base_z::Amcl * amcl_;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbWaitPose>();
    configure_orthogonal<OrNavigation, CbWaitActionServer>(std::chrono::milliseconds(10000));
    configure_orthogonal<OrNavigation, CbWaitNav2Nodes>(std::vector<Nav2Nodes>{
      Nav2Nodes::PlannerServer, Nav2Nodes::ControllerServer, Nav2Nodes::RecoveriesServer,
      Nav2Nodes::BtNavigator, Nav2Nodes::MapServer});
  }

  void runtimeConfigure()
  {
    ClMoveBaseZ * navClient;
    getOrthogonal<OrNavigation>()->requiresClient(navClient);
    amcl_ = navClient->getComponent<Amcl>();
  }

  void onEntry() { sendInitialPoseEstimation(); }

  void sendInitialPoseEstimation()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped initialposemsg;
    bool useSimTime = getNode()->get_parameter("use_sim_time").as_bool();
    //getNode()->set_parameter("use_sim_time",true);

    initialposemsg.header.stamp = getNode()->now();
    initialposemsg.header.frame_id = "map";

    initialposemsg.pose.pose.position.x = 3.415412425994873;
    initialposemsg.pose.pose.position.y = 2.0;
    initialposemsg.pose.pose.position.z = 0;

    initialposemsg.pose.pose.orientation.x = 0;
    initialposemsg.pose.pose.orientation.y = 0;
    initialposemsg.pose.pose.orientation.z = 1;
    initialposemsg.pose.pose.orientation.w = 0;

    //z: 0.9999985465626609
    // w: 0.00170495529732811

    initialposemsg.pose.covariance = {
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
      0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.06853891909122467};

    amcl_->setInitialPose(initialposemsg);
  }

  void onExit() {}
};
}  // namespace sm_aws_warehouse_navigation
