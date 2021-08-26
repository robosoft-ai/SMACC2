#include <smacc/client_base_components/cp_ros_control_interface.h>

#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>

namespace smacc
{
namespace components
{
using namespace controller_manager_msgs;

CpRosControlInterface::CpRosControlInterface()
{
}

CpRosControlInterface::~CpRosControlInterface()
{
}

void CpRosControlInterface::onInitialize()
{
    srvListControllers = nh_.serviceClient<ListControllers>(*serviceName_);
    srvListControllersTypes = nh_.serviceClient<ListControllerTypes>(*serviceName_);

    srvLoadController = nh_.serviceClient<LoadController>(*serviceName_);
    srvUnloadController = nh_.serviceClient<UnloadController>(*serviceName_);
    srvReloadControllerLibraries = nh_.serviceClient<ReloadControllerLibraries>(*serviceName_);
    srvSwitchControllers = nh_.serviceClient<SwitchController>(*serviceName_);
}

std::vector<controller_manager_msgs::ControllerState> CpRosControlInterface::listControllers()
{
    ListControllers::Request req;
    ListControllers::Response res;

    srvListControllers.call(req, res);
    return res.controller;
}

std::vector<ControllerTypeInfo> CpRosControlInterface::listControllerTypes()
{
    ListControllerTypes::Request req;
    ListControllerTypes::Response res;
    srvListControllersTypes.call(req, res);

    std::vector<ControllerTypeInfo> ret;
    for (int i = 0; i < res.types.size(); i++)
    {
        ControllerTypeInfo entry;
        entry.type = res.types[i];
        entry.baseClass = res.base_classes[i];

        ret.push_back(entry);
    }

    return ret;
}

bool CpRosControlInterface::loadController(std::string name)
{
    LoadController::Request req;
    LoadController::Response res;
    req.name = name;

    srvLoadController.call(req, res);
    return res.ok;
}

bool CpRosControlInterface::unloadController(std::string name)
{
    UnloadController::Request req;
    UnloadController::Response res;

    req.name = name;
    srvUnloadController.call(req, res);
    return res.ok;
}

bool CpRosControlInterface::reloadControllerLibraries(bool forceKill)
{
    ReloadControllerLibraries::Request req;
    ReloadControllerLibraries::Response res;
    req.force_kill = forceKill;

    srvReloadControllerLibraries.call(req, res);

    return res.ok;
    //srvReloadControllerLibraries
}

bool CpRosControlInterface::switchControllers(std::vector<std::string> start_controllers,
                                              std::vector<std::string> stop_controllers,
                                              Strictness strictness)
{
    SwitchController::Request req;
    SwitchController::Response res;

    req.start_controllers = start_controllers;
    req.stop_controllers = stop_controllers;
    req.strictness = strictness;

    srvSwitchControllers.call(req, res);

    return res.ok;
}
}
}
