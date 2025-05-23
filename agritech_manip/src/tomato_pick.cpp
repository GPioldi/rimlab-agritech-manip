// #include "agritech_manip/TomatoPickNode.h"
#include <agritech_manip/tomato_pick_node.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node =
            std::make_shared<agritech_manip::TomatoPickNode>("tomato_pick");
        // RCLCPP_INFO(node->getLogger(), "node started");
        node->spin();
    } catch (std::exception& e) {
        std::cerr << "exception in " << __FILE__ << ": " << e.what()
                  << std::endl;
        return 1;
    }
    return 0;
}
