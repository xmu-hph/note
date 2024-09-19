# 1.问题定义
`c++`可以通过`pybind11`打包为库函数供`python`使用与交互。比如`python`发出指令，`c++`环境执行内部计算。但是在某些情况下，我们可能会需要`c++`使用`python`主程序中的变量以及`python`中定义好的函数来执行计算。这也可以通过`pybind11`来实现。
# 2.解决方案
`python`的一次交互可能需要`c++`多次执行同一函数。对于函数，不需要每次都导入一遍；对于环境中会发生变化的对象，需要每次计算都导入一遍。所以我们可以针对不同的对象进行不同的导入设计。
对于函数，可以设置为类对象所共享的全局变量，因为不会对他进行修改，同时只在第一次创建对象时导入，不必每个对象都导入一遍。
```cpp
#include "vehicle/vehicle.h"
#include "engine/engine.h"
#include <pybind11/pybind11.h>
#include <iostream>
#include <limits>
#include <random>
// 静态成员变量初始化
std::shared_ptr<pybind11::module> CityFlow::Vehicle::pyModule = nullptr;
std::shared_ptr<pybind11::object> CityFlow::Vehicle::add_func = nullptr;
std::shared_ptr<pybind11::object> CityFlow::Vehicle::pymodel = nullptr;
namespace CityFlow {
    Vehicle::Vehicle(const Vehicle &vehicle, Flow *flow)
        : vehicleInfo(vehicle.vehicleInfo), controllerInfo(this, vehicle.controllerInfo),
          laneChangeInfo(vehicle.laneChangeInfo), buffer(vehicle.buffer), priority(vehicle.priority),
          id(vehicle.id), engine(vehicle.engine),
          laneChange(std::make_shared<SimpleLaneChange>(this, *vehicle.laneChange)),
          flow(flow){
        enterTime = vehicle.enterTime;
        // 检查是否已经初始化静态成员变量
        if (!pyModule && !add_func && !pymodel) {
            pyModule = std::make_shared<pybind11::module>(pybind11::module::import("car_follow"));
            add_func = std::make_shared<pybind11::object>(pyModule->attr("customfollow"));
            pymodel = std::make_shared<pybind11::object>(pyModule->attr("model"));
            double test = (*add_func)(2, 2, 2, 2, 3, 3, 3, "nihao").cast<double>();
            std::cout << "No collision speed: " << test << std::endl;
            std::cout << "Successfully loaded PyTorch model." << std::endl;
        }
    }
    // should be move to seperate CarFollowing (Controller?) class later?
    double Vehicle::getCarFollowSpeed(double interval) {
        Vehicle *leader = getLeader();
        if (leader == nullptr) return hasSetCustomSpeed() ? buffer.customSpeed : vehicleInfo.maxSpeed;
        // 从 Python 中获取主函数的变量
        pybind11::object py_globals = pybind11::module::import("__main__").attr("__dict__"); // 获取 Python 主函数的全局字典
        pybind11::object model = py_globals["model"];  // 假设 "model" 是主函数中的模型变量
        double v = (*add_func)(leader->getSpeed(), leader->getMaxNegAcc(), vehicleInfo.speed,
                                       vehicleInfo.maxNegAcc, controllerInfo.gap, interval, 0, (model)).cast<double>();
        if (hasSetCustomSpeed())
            return min2double(buffer.customSpeed, v);
        // safe distance
        // get relative decel (mimic real scenario)
        double assumeDecel = 0, leaderSpeed = leader->getSpeed();
        if (vehicleInfo.speed > leaderSpeed) {
            assumeDecel = vehicleInfo.speed - leaderSpeed;
        }
        v = min2double(v, getNoCollisionSpeed(leader->getSpeed(), leader->getUsualNegAcc(), vehicleInfo.speed,
                                              vehicleInfo.usualNegAcc, controllerInfo.gap, interval,
                                              vehicleInfo.minGap
                                              ));
        v = min2double(v,
                       (controllerInfo.gap + (leaderSpeed + assumeDecel / 2) * interval -
                        vehicleInfo.speed * interval / 2) / (vehicleInfo.headwayTime + interval / 2));

        return v;
    }
```
对于主函数中的变量，由于是动态对象，会随着程序的演进不断变化，所以应在函数每次执行时导入一遍。
