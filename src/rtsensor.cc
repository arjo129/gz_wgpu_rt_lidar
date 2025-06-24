#include "rtsensor.hh"
#include <gz/sensors/SensorFactory.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <sdf/Element.hh>

using namespace rtsensor;

//////////////////////////////////////////////////
RtSensor::RtSensor()
{
  gzdbg << "RtSensor: Constructor called" << std::endl;
}

//////////////////////////////////////////////////
RtSensor::~RtSensor()
{
  gzdbg << "RtSensor: Destructor called" << std::endl;
}

//////////////////////////////////////////////////
bool RtSensor::Load(const sdf::Sensor &_sdf)
{
  gzdbg << "RtSensor: Loading sensor configuration" << std::endl;
  
  // Load base sensor properties
  if (!gz::sensors::Sensor::Load(_sdf))
  {
    gzerr << "RtSensor: Failed to load base sensor properties" << std::endl;
    return false;
  }

  // Get the custom sensor type from SDF
  //auto customType = gz::sensors::customType(_sdf);
  std::string customType;
  if (_sdf.Element()->HasAttribute("gz:type"))
    customType = _sdf.Element()->Get<std::string>("gz:type");
  else
    customType = "";
  gzdbg << "RtSensor: Custom sensor type: " << customType << std::endl;
  
  // Determine sensor type
  if (customType == "rt_lidar")
  {
    this->sensorType = SensorType::LIDAR;
    gzdbg << "RtSensor: Configured as LIDAR sensor" << std::endl;
  }
  else if (customType == "rt_camera")
  {
    this->sensorType = SensorType::CAMERA;
    gzdbg << "RtSensor: Configured as CAMERA sensor" << std::endl;
  }
  else
  {
    gzerr << "RtSensor: Unknown custom sensor type: " << customType << std::endl;
    return false;
  }

  // Load basic sensor configuration
  this->config.topic = this->Topic();
  this->config.updateRate = this->UpdateRate();
  this->config.alwaysOn = this->IsActive();
  
  // Parse sensor-specific configuration
  auto sdfRoot = _sdf.Element();
  if (sdfRoot)
  {
    if (this->sensorType == SensorType::LIDAR)
    {
      auto lidarElement = sdfRoot->FindElement("gz:rt_lidar");
      if (lidarElement)
      {
        this->ParseLidarConfig(lidarElement);
      }
    }
    else if (this->sensorType == SensorType::CAMERA)
    {
      auto cameraElement = sdfRoot->FindElement("gz:rt_camera");
      if (cameraElement)
      {
        this->ParseCameraConfig(cameraElement);
      }
    }
  }

  // Print all configuration for debugging
  this->PrintConfiguration();

  gzdbg << "RtSensor: Successfully loaded sensor configuration" << std::endl;
  return true;
}

//////////////////////////////////////////////////
bool RtSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  // Simple update - just print that we're updating
  static int updateCount = 0;
  updateCount++;
  
  if (updateCount % 100 == 0) // Print every 100 updates to avoid spam
  {
    gzdbg << "RtSensor: Update #" << updateCount 
          << " at time " << std::chrono::duration_cast<std::chrono::milliseconds>(_now).count() 
          << "ms" << std::endl;
  }

  return true;
}

//////////////////////////////////////////////////
void RtSensor::SetPose(const gz::math::Pose3d &_pose)
{
  this->currentPose = _pose;
}

//////////////////////////////////////////////////
const gz::math::Pose3d &RtSensor::Pose() const
{
  return this->currentPose;
}

//////////////////////////////////////////////////
RtSensor::SensorType RtSensor::GetSensorType() const
{
  return this->sensorType;
}

//////////////////////////////////////////////////
void RtSensor::ParseLidarConfig(const sdf::ElementPtr &_sdf)
{
  gzdbg << "RtSensor: Parsing LIDAR configuration" << std::endl;
}

//////////////////////////////////////////////////
void RtSensor::ParseCameraConfig(const sdf::ElementPtr &_sdf)
{
  gzdbg << "RtSensor: Parsing CAMERA configuration" << std::endl;
}

//////////////////////////////////////////////////
void RtSensor::PrintConfiguration()
{
  gzmsg << "\n======= RT SENSOR CONFIGURATION =======" << std::endl;
  gzmsg << "Sensor Name: " << this->Name() << std::endl;
  gzmsg << "Sensor Type: " << (this->sensorType == SensorType::LIDAR ? "LIDAR" : "CAMERA") << std::endl;

  if (this->sensorType == SensorType::LIDAR)
  {
    gzmsg << "\n--- LIDAR Configuration ---" << std::endl;
    gzmsg << "FOV: " << this->config.lidar.fov << std::endl;
  }
  else if (this->sensorType == SensorType::CAMERA)
  {
    gzmsg << "\n--- CAMERA Configuration ---" << std::endl;
    gzmsg << "FOV: " << this->config.camera.fov << std::endl;
  }
}

// Register this sensor with the sensor factory
GZ_ADD_PLUGIN(RtSensor, gz::sensors::Sensor)

// Register the sensor type
GZ_ADD_PLUGIN_ALIAS(RtSensor, "rtsensor::RtSensor")
