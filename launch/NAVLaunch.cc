#include "launch/NAVLaunch.h"

#include "yaml-cpp/yaml.h"

#include <string>
#include <thread>
#include <vector>

namespace nav {
namespace launch {

NAVLaunch::NAVLaunch(std::vector<std::string> dagFiles)
    : dagFiles_{std::move(dagFiles)}
{
    // TODO: let's make sure each file is a yaml file and passes some basic checks
}

//void NAVLaunch::launch()
//{
//    for (auto const& dagFile : dagFiles_) {
//        try {
//            YAML::Node config = YAML::LoadFile(dagFile);
//
//            if (config.IsNull()) {
//                std::cerr << "Failed to load the config file!" << std::endl;
//                //                return -1;
//            }
//
//            if (!config["ModuleConfig"]) {
//                std::cerr << "ModuleConfig not found in the YAML file!" << std::endl;
//                //                return -1;
//            }
//            auto moduleName = config["ModuleConfig"]["ModuleName"].as<std::string>();
//
//            std::cout << "ModuleName: " << moduleName << std::endl;
//            std::cout << "ModuleLibrary: " << config["ModuleConfig"]["ModuleLibrary"].as<std::string>() << std::endl;
//
//            auto moduleLaunchConfig = ModuleLaunchConfig{};
//            moduleLaunchConfig.moduleName_ = moduleName;
//            moduleLaunchConfig.moduleSharedLibraryObjectFilePath_ =
//                config["ModuleConfig"]["ModuleLibrary"].as<std::string>();
//            moduleLaunchConfig.isSingleThreaded_ = true;
//
//            modulesMap_[std::move(moduleName)] = std::move(moduleLaunchConfig);
//
//        } catch (const YAML::Exception& e) {
//            std::cerr << "Error parsing YAML: " << e.what() << std::endl;
//            //            return -1;
//        }
//    }
//
//    for (const auto& [moduleName, moduleLaunchConfig] : modulesMap_) {
//        if (moduleLaunchConfig.isSingleThreaded_) {
//            auto executorThread = launchModule<rclcpp::executors::SingleThreadedExecutor>(moduleLaunchConfig);
//            executorThreads_.push_back(std::move(executorThread));
//        } else {
//            auto executorThread = launchModule<rclcpp::executors::MultiThreadedExecutor>(moduleLaunchConfig);
//            executorThreads_.push_back(std::move(executorThread));
//        }
//    }
//    for (auto& t : executorThreads_) {
//        t.join();
//    }
//}
//
//// ROS2SystemInit::ROS2SystemInit(std::unique_ptr<rclcpp_components::ComponentManager> componentManager,
////                                std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor)
////     : componentManager_{std::move(componentManager)}
////     , executor_{std::move(executor)}
////{}
////
//// void ROS2SystemInit::StartExecutor() {
////     executor_->add_node(componentManager_);
//// }
} // namespace launch
} // namespace nav
