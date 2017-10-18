/**
 * @file /kobuki_controller_tutorial/src/nodelet.cpp
 *
 * @brief Nodelet implementation of the BumpBlinkController
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_drive/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <string>
#include <ecl/threads/thread.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "kobuki_drive/drive_controller.hpp"


namespace kobuki
{

/**
 * @brief Nodelet-wrapper of the DriveController class
 */
class DriveControllerNodelet : public nodelet::Nodelet
{
public:
  DriveControllerNodelet(): shutdown_requested_(false){};
  ~DriveControllerNodelet()
  {
    shutdown_requested_ = true;
    NODELET_DEBUG_STREAM("Waiting for update thread to finish. [" << name_ << "]");
    update_thread_.join();
    NODELET_INFO_STREAM("Controller is shutting down. [" << name_ << "]");
  }

  /**
   * @brief Initialise the nodelet
   *
   * This function is called, when the nodelet manager loads the nodelet.
   */
  virtual void onInit()
  {
    ros::NodeHandle nh_priv = this->getPrivateNodeHandle();

    // resolve node(let) name
    std::string name = nh_priv.getUnresolvedNamespace();
    int pos = name.find_last_of('/');
    name_ = name.substr(pos + 1);

    NODELET_INFO_STREAM("Initialising nodelet ... [" << name_ << "]");
    controller_.reset(new DriveController(nh_priv, name_));

    nh_priv.param("update_rate", update_rate_, 10.0);
    NODELET_INFO_STREAM("Controller will spin at " << update_rate_ << " hz. [" << name_ << "]");

    // Initialises the controller
    if (controller_->init())
    {
      NODELET_INFO_STREAM("Nodelet initialised. Spinning up update thread. [" << name_ << "]");
      update_thread_.start(&DriveControllerNodelet::update, *this);
    }
    else
    {
      NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name_ << "]");
    }
  }

private:
  /// Pointer to the random walker controller
  boost::shared_ptr<DriveController> controller_;
  /// Spin rate for the update thread
  double update_rate_;
  /// Node(let) name
  std::string name_;
  /// Update thread for publishing velocity and LED commands
  ecl::Thread update_thread_;
  /// Flag for stopping the update thread
  bool shutdown_requested_;

  /// Method for update thread
  void update()
  {
    ros::Rate spin_rate(update_rate_);
    while (ros::ok() && !shutdown_requested_)
    {
      controller_->spin();
      spin_rate.sleep();
    }
  }
};

} // namespace kobuki

PLUGINLIB_EXPORT_CLASS(kobuki::DriveControllerNodelet,
                       nodelet::Nodelet);
