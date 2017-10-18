#include <string>
#include <ecl/threads/thread.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "kobuki_walker/walker_controller.h"

namespace kobuki {

    class WalkerControllerNodelet : public nodelet::Nodelet {

        public:
            WalkerControllerNodelet(): shutdown_requested(false){};
            ~WalkerControllerNodelet();
            virtual void onInit();

        private:
            boost::shared_ptr<WalkerController> controller;
            double update_rate;
            std::string name;
            ecl::Thread update_thread;
            bool shutdown_requested;
            void update();


    };

}

PLUGINLIB_EXPORT_CLASS(kobuki::WalkerControllerNodelet,
                       nodelet::Nodelet);