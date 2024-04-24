#pragma once

#include "component/Component.h"
#include "component/ROSComponentFactory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "class_loader/class_loader.hpp"

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace av {
namespace launch {

class NAVLaunch
{
public:
    NAVLaunch(std::vector<std::string> dagFiles);

    virtual void launch() = 0;

protected:
    struct ModuleLaunchConfig
    {
        std::string moduleName_;
        std::string moduleSharedLibraryObjectFilePath_;
        bool isSingleThreaded_;
    };
//    template<class rclcpp_Executor>
//    virtual std::thread launchModule(const ModuleLaunchConfig& moduleLaunchConfig) = 0;

    //    rclcpp::Logger logger_;
    std::vector<std::string> dagFiles_;
    std::mutex launchModuleLock_;
    std::vector<std::unique_ptr<rclcpp::Executor>> executors_;
    std::vector<std::thread> executorThreads_;
    rclcpp::NodeOptions options_;
    std::vector<std::unique_ptr<class_loader::ClassLoader>> loaders_;
    std::vector<rclcpp_components::NodeInstanceWrapper> nodeWrappers_;
    std::map<std::string, ModuleLaunchConfig> modulesMap_;
};

//template<class rclcpp_Executor>
//std::thread NAVLaunch::launchModule(const ModuleLaunchConfig& moduleLaunchConfig)
//{
//    std::lock_guard<std::mutex> lockGuard(launchModuleLock_);
//    std::unique_ptr<rclcpp_Executor> executor = std::make_unique<rclcpp_Executor>();
//    executors_.push_back(std::move(executor));
//    std::thread t1([&]() {
//        auto& executor = executors_.back();
//        auto loader =
//            std::make_unique<class_loader::ClassLoader>(moduleLaunchConfig.moduleSharedLibraryObjectFilePath_);
//
//        auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
//
//        // TODO: WE ARE HERE NOW
//        // TODO: WE ARE HERE NOW
//        auto configs = loader->getAvailableClasses<component::ComponentConfig>();
//
//        for (const auto& clazz : classes) {
//            //            RCLCPP_INFO(logger, "Instantiate class %s", clazz.c_str());
//            auto nodeFactory = loader->createInstance<rclcpp_components::NodeFactory>(clazz);
//            auto wrapper = nodeFactory->create_node_instance(options_);
//            auto node = wrapper.get_node_base_interface();
//            nodeWrappers_.push_back(wrapper);
//            executor->add_node(node);
//        }
//        loaders_.push_back(std::move(loader));
//
//        executor->spin();
//
//        for (auto wrapper : nodeWrappers_) {
//            executor->remove_node(wrapper.get_node_base_interface());
//        }
//        nodeWrappers_.clear();
//
//        rclcpp::shutdown();
//    });
//    return t1;
//    //    executorThread_.push_back(std::move(t1));
//}

} // namespace launch
} // namespace nav
