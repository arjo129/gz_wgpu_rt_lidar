#include "rtsensorsystem.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Util.hh>
#include <sdf/Sensor.hh>

using namespace rtsensor;

void RtSensorSystem::PreUpdate(const gz::sim::UpdateInfo &,
                               gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachNew<gz::sim::components::CustomSensor,
               gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *_custom,
        const gz::sim::components::ParentEntity *_parent)->bool
      {
        auto sensorScopedName = gz::sim::removeParentScope(
            gz::sim::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName);

        if (data.Topic().empty())
        {
          std::string topic = gz::sim::scopedName(_entity, _ecm) + "/rt_sensor";
          data.SetTopic(topic);
        }

        // Directly instantiate and load the sensor
        auto sensor = std::make_shared<RtSensor>();
        if (!sensor->Load(data))
        {
          gzerr << "Failed to load RtSensor [" << sensorScopedName << "]" << std::endl;
          return false;
        }

        auto parentName = _ecm.Component<gz::sim::components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // No SensorTopic yet

        this->entitySensorMap.insert(std::make_pair(_entity, std::move(sensor)));
        return true;
      });
}

void RtSensorSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                                const gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused)
  {
    for (auto &[entity, sensor] : this->entitySensorMap)
    {
      // Optionally update pose here if needed
      auto baseSensor = std::dynamic_pointer_cast<gz::sensors::Sensor>(sensor);
      if (baseSensor)
      {
        baseSensor->Update(_info.simTime, false);
      }
      else
      {
        sensor->Update(_info.simTime);
        gzerr << "Error casting RtSensor to base Sensor class." << std::endl;
      }
    }
  }
  this->RemoveSensorEntities(_ecm);
}

void RtSensorSystem::RemoveSensorEntities(const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<gz::sim::components::CustomSensor>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *)->bool
      {
        if (this->entitySensorMap.erase(_entity) == 0)
        {
          gzerr << "Internal error, missing RtSensor for entity ["
                << _entity << "]" << std::endl;
        }
        return true;
      });
}

GZ_ADD_PLUGIN(RtSensorSystem, gz::sim::System,
  RtSensorSystem::ISystemPreUpdate,
  RtSensorSystem::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(RtSensorSystem, "rtsensor::RtSensorSystem")
