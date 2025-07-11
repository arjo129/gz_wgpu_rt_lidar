#include <gz/common/Console.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <string>

#include "rtsensor.hh"

using namespace rtsensor;

  //////////////////////////////////////////////////
  RtSensor::RtSensor()
  {
    gzdbg << "[RtSensor] Constructor called" << std::endl;
  }

  //////////////////////////////////////////////////
  RtSensor::~RtSensor()
  {
    gzdbg << "[RtSensor] Destructor called" << std::endl;
  }

  //////////////////////////////////////////////////
  bool RtSensor::Load(const sdf::Sensor &_sdf)
  {
    gzdbg << "[RtSensor] Loading sensor configuration" << std::endl;

    // Load base sensor properties inherited from gz::sensors::Sensor
    if (!gz::sensors::Sensor::Load(_sdf))
    {
      gzerr << "[RtSensor] Failed to load base sensor properties" << std::endl;
      return false;
    }

    this->topic = this->Topic();

    // Get the custom sensor type from SDF.
    // This is expected to be defined as <sensor type="custom" gz:type="rt_camera"> or "rt_lidar"
    std::string customType;
    if (_sdf.Element()->HasAttribute("gz:type"))
    {
      customType = _sdf.Element()->Get<std::string>("gz:type");
    }
    else
    {
      gzerr << "[RtSensor] Missing 'gz:type' attribute for custom sensor." << std::endl;
      return false;
    }

    gzdbg << "[RtSensor] Detected custom sensor type: " << customType << std::endl;

    // Determine sensor type and parse type-specific parameters
    if (customType == "rt_lidar")
    {
      this->sensorType = SensorType::LIDAR;
      // Parse LiDAR specific configuration
      auto lidarElement = _sdf.Element()->FindElement("gz:rt_lidar");
      if (lidarElement)
      {
        if (lidarElement->HasElement("fov"))
        {
          this->fov = lidarElement->Get<double>("fov");
          gzdbg << "[RtSensor] LiDAR FOV set to: " << this->fov << std::endl;
        }
      }
    }
    else if (customType == "rt_camera")
    {
      this->sensorType = SensorType::CAMERA;
      // Parse Camera specific configuration
      auto cameraElement = _sdf.Element()->FindElement("gz:rt_camera");
      if (cameraElement)
      {
        if (cameraElement->HasElement("fov"))
        {
          this->fov = cameraElement->Get<double>("fov");
          gzdbg << "[RtSensor] Camera FOV set to: " << this->fov << std::endl;
        }
      }
    }
    else
    {
      gzerr << "[RtSensor] Unknown custom sensor type: [" << customType << "]" << std::endl;
      return false;
    }

    gzmsg << "[RtSensor] Successfully loaded sensor [" << this->Name() << "] of type ["
          << (this->sensorType == SensorType::LIDAR ? "LIDAR" : "CAMERA")
          << "], topic: [" << this->topic << "], FOV: [" << this->fov << "]" << std::endl;
    return true;
  }

  const std::string& RtSensor::TopicName() const
  {
    return this->topic;
  }

  //////////////////////////////////////////////////
  bool RtSensor::Update(const std::chrono::steady_clock::duration &_now)
  {
    gzdbg << "[RtSensor] Update called for sensor [" << this->Name() << "] at sim time: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(_now).count() << "ms" << std::endl;
    return true;
  }

  //////////////////////////////////////////////////
  void RtSensor::SetParentEntityName(const std::string& _parentName)
  {
    this->parentEntityName = _parentName;
  }

  //////////////////////////////////////////////////
  const std::string& RtSensor::ParentEntityName() const
  {
    return this->parentEntityName;
  }

  //////////////////////////////////////////////////
  RtSensor::SensorType RtSensor::Type() const
  {
    return this->sensorType;
  }

  //////////////////////////////////////////////////
  double RtSensor::FOV() const
  {
    return this->fov;
  }

  //////////////////////////////////////////////////
  const gz::math::Pose3d &RtSensor::Pose() const
  {
    return this->currentPose;
  }

  //////////////////////////////////////////////////
  void RtSensor::SetPose(const gz::math::Pose3d &_pose)
  {
    this->currentPose = _pose;
  }
