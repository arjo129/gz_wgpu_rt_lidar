#pragma once

#include <gz/sim/System.hh>
#include <gz/sensors/Sensor.hh>
#include <unordered_map>
#include <memory>
#include "rtsensor.hh"

namespace rtsensor
{
  class RtSensorSystem :
    public gz::sim::System,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) final;

    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) final;

    private: void RemoveSensorEntities(const gz::sim::EntityComponentManager &_ecm);

    private: std::unordered_map<gz::sim::Entity, std::shared_ptr<RtSensor>> entitySensorMap;
  };
}
