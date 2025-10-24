#include <gz/common/Console.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <string>
#include <chrono>
#include <cmath>

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
bool RtSensor::Load(const sdf::Sensor & _sdf)
{
  gzdbg << "[RtSensor] Loading sensor configuration" << std::endl;

  // Load base sensor properties inherited from gz::sensors::Sensor
  if (!gz::sensors::Sensor::Load(_sdf)) {
    gzerr << "[RtSensor] Failed to load base sensor properties" << std::endl;
    return false;
  }

  this->config.topic = this->Topic();
  this->config.updateRate = this->UpdateRate();
  this->config.alwaysOn = this->IsActive();
  this->config.visualize = _sdf.Element()->Get < bool > ("visualize", false).first;

  // Get the custom sensor type from SDF.
  // This is expected to be defined as <sensor type="custom" gz:type="rt_camera"> or "rt_lidar"
  std::string customType;
  if (_sdf.Element()->HasAttribute("gz:type")) {
    customType = _sdf.Element()->Get < std::string > ("gz:type");
  } else {
    gzerr << "[RtSensor] Missing 'gz:type' attribute for custom sensor." << std::endl;
    return false;
  }

  gzdbg << "[RtSensor] Detected custom sensor type: " << customType << std::endl;

  // Determine sensor type and parse type-specific parameters
  if (customType == "rt_lidar") {
    this->sensorType = SensorType::LIDAR;
    // Parse LiDAR specific configuration
    auto lidarElement = _sdf.Element()->FindElement("gz:rt_lidar");
    if (lidarElement) {
      this->LidarConfig(lidarElement);
    } else {
      gzerr << "[RtSensor] Missing required <gz:rt_lidar> element for rt_lidar sensor [" <<
        this->Name() << "]. Using default parameters." << std::endl;
    }
  } else if (customType == "rt_camera") {
    this->sensorType = SensorType::CAMERA;
    // Parse Camera specific configuration
    auto cameraElement = _sdf.Element()->FindElement("gz:rt_camera");
    if (cameraElement) {
      this->CameraConfig(cameraElement);
    } else {
      gzerr << "[RtSensor] Missing required <gz:rt_camera> element for rt_camera sensor [" <<
        this->Name() << "]. Using default parameters." << std::endl;
    }
  } else {
    gzerr << "[RtSensor] Unknown custom sensor type: [" << customType << "]" << std::endl;
    return false;
  }

  gzmsg << "[RtSensor] Successfully loaded sensor [" << this->Name() << "] of type ["
        << (this->sensorType == SensorType::LIDAR ? "LIDAR" : "CAMERA")
        << "], topic: [" << this->config.topic << "]" << std::endl;
  return true;
}

//////////////////////////////////////////////////
const std::string & RtSensor::TopicName() const
{
  return this->config.topic;
}

//////////////////////////////////////////////////
bool RtSensor::Update(const std::chrono::steady_clock::duration & _now)
{
  gzdbg << "[RtSensor] Update called for sensor [" << this->Name() << "] at sim time: "
        << std::chrono::duration_cast < std::chrono::milliseconds >
    (_now).count() << "ms" << std::endl;
  return true;
}

//////////////////////////////////////////////////
void RtSensor::SetParentEntity(const gz::sim::Entity & _parentEntity)
{
  this->parentEntity = _parentEntity;
}

//////////////////////////////////////////////////
gz::sim::Entity RtSensor::ParentEntity() const
{
  return this->parentEntity;
}

//////////////////////////////////////////////////
RtSensor::SensorType RtSensor::Type() const
{
  return this->sensorType;
}


//////////////////////////////////////////////////
const gz::math::Pose3d & RtSensor::Pose() const
{
  return this->currentPose;
}

//////////////////////////////////////////////////
void RtSensor::SetPose(const gz::math::Pose3d & _pose)
{
  this->currentPose = _pose;
}

//////////////////////////////////////////////////
void RtSensor::LidarConfig(const sdf::ElementPtr & _sdf)
{
  gzdbg << "RtSensor: Parsing LIDAR configuration" << std::endl;

  // Parse scan configuration
  if (_sdf->HasElement("scan")) {
    auto scanElem = _sdf->GetElement("scan");
    this->config.lidar.num_lasers = scanElem->Get < unsigned int > ("lasers", 16).first;
    this->config.lidar.num_steps = scanElem->Get < unsigned int > ("steps", 720).first;

    if (scanElem->HasElement("vertical")) {
      auto vertElem = scanElem->GetElement("vertical");
      this->config.lidar.min_vertical_angle = vertElem->Get < double > ("min_angle", -M_PI).first;
      this->config.lidar.max_vertical_angle = vertElem->Get < double > ("max_angle", M_PI).first;
      if (this->config.lidar.num_lasers > 1) {
        this->config.lidar.step_vertical_angle = (this->config.lidar.max_vertical_angle -
          this->config.lidar.min_vertical_angle) / (this->config.lidar.num_lasers - 1);
      } else {
        this->config.lidar.step_vertical_angle = 0.0;
      }
    }

    if (scanElem->HasElement("horizontal")) {
      auto horzElem = scanElem->GetElement("horizontal");
      this->config.lidar.min_horizontal_angle = horzElem->Get < double > ("min_angle", -M_PI).first;
      this->config.lidar.max_horizontal_angle = horzElem->Get < double > ("max_angle", M_PI).first;
      if (this->config.lidar.num_steps > 1) {
        this->config.lidar.step_horizontal_angle = (this->config.lidar.max_horizontal_angle -
          this->config.lidar.min_horizontal_angle) / (this->config.lidar.num_steps - 1);
      } else {
        this->config.lidar.step_horizontal_angle = 0.0;
      }
    }
  }

  // Parse range configuration
  if (_sdf->HasElement("range")) {
    auto rangeElem = _sdf->GetElement("range");
    this->config.lidar.min_range = rangeElem->Get < double > ("min", 0.1).first;
    this->config.lidar.max_range = rangeElem->Get < double > ("max", 100.0).first;
  }

  // Parse noise configuration
  if (_sdf->HasElement("noise")) {
    auto noiseElem = _sdf->GetElement("noise");
    this->config.noise.type = noiseElem->Get < std::string > ("type", "gaussian").first;
    this->config.noise.mean = noiseElem->Get < double > ("mean", 0.0).first;
    this->config.noise.stddev = noiseElem->Get < double > ("stddev", 0.01).first;
  }
}

//////////////////////////////////////////////////
void RtSensor::CameraConfig(const sdf::ElementPtr & _sdf)
{
  gzdbg << "RtSensor: Parsing CAMERA configuration" << std::endl;

  if (_sdf->HasElement("image")) {
    auto imageElem = _sdf->GetElement("image");
    this->config.camera.width = imageElem->Get < unsigned int > ("width", 256).first;
    this->config.camera.height = imageElem->Get < unsigned int > ("height", 256).first;
  }

  // Parse camera configuration
  if (_sdf->HasElement("camera")) {
    auto camElem = _sdf->GetElement("camera");
    this->config.camera.horizontalFov = camElem->Get < double > ("horizontal_fov", 60.0).first;
    this->config.camera.verticalFov = camElem->Get < double > ("vertical_fov", 60.0).first;
    this->config.camera.nearClip = camElem->Get < double > ("near_clip", 0.1).first;
    this->config.camera.farClip = camElem->Get < double > ("far_clip", 100.0).first;
  }

  // Parse noise configuration
  if (_sdf->HasElement("noise")) {
    auto noiseElem = _sdf->GetElement("noise");
    this->config.noise.type = noiseElem->Get < std::string > ("type", "gaussian").first;
    this->config.noise.mean = noiseElem->Get < double > ("mean", 0.0).first;
    this->config.noise.stddev = noiseElem->Get < double > ("stddev", 0.01).first;
  }
}
