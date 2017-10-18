#include "nodelet.h"

namespace kobuki {

    WalkerControllerNodelet::~WalkerControllerNodelet() {
        shutdown_requested = true;
        NODELET_DEBUG_STREAM("Waiting for update thread to finish. [" << name << "]");
        update_thread.join();
        NODELET_INFO_STREAM("Controller is shutting down. [" << name << "]");
    }

    void WalkerControllerNodelet::onInit() {
        ros::NodeHandle nh_priv = this->getPrivateNodeHandle();

        // resolve node(let) name
        std::string name = nh_priv.getUnresolvedNamespace();
        int pos = name.find_last_of('/');
        name = name.substr(pos + 1);

        NODELET_INFO_STREAM("Initialising nodelet ... [" << name << "]");
        controller.reset(new WalkerController(nh_priv, name));

        nh_priv.param("update_rate", update_rate, 10.0);
        NODELET_INFO_STREAM("Controller will spin at " << update_rate << " hz. [" << name << "]");

        // Initialises the controller
        if (controller->init()) {
          NODELET_INFO_STREAM("Nodelet initialised. Spinning up update thread. [" << name << "]");
          update_thread.start(&WalkerControllerNodelet::update, *this);
        } else {
          NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
        }
    }

    void WalkerControllerNodelet::update() {
        ros::Rate spin_rate(update_rate);
        while (ros::ok() && !shutdown_requested) {
            controller->spin();
            spin_rate.sleep();
        }
    }

}