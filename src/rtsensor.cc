#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <string>
#include <chrono>

namespace rtsensor
{
  class RtSensor : public gz::sensors::Sensor
  {
    /// \brief Sensor type enumeration.
    public: enum class SensorType
    {
      CAMERA,
      LIDAR
    };

    /// \brief Constructor.
    public: RtSensor();

    /// \brief Destructor.
    public: ~RtSensor();

    /// \brief Load the sensor with SDF parameters.
    public: bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Update the sensor and generate data.
    public: bool Update(
      const std::chrono::steady_clock::duration &_now) override;

    /// \brief Set the current pose of the sensor in world coordinates.
    public: void SetPose(const gz::math::Pose3d &_pose);

    /// \brief Get the latest world pose of the sensor.
    public: const gz::math::Pose3d &Pose() const;

    /// \brief Get the sensor type.
    public: SensorType Type() const;

    /// \brief Get the Field of View (FOV) configured for the sensor.
    public: double FOV() const;

    /// \brief Set the name of the parent entity (e.g., link or model) this sensor is attached to.
    public: void SetParentEntityName(const std::string& _parentName);

    /// \brief Get the name of the parent entity.
    public: const std::string& ParentEntityName() const;

    /// \brief Sensor type (camera or lidar).
    private: SensorType sensorType{SensorType::CAMERA};

    /// \brief Current sensor pose in world coordinates.
    private: gz::math::Pose3d currentPose{gz::math::Pose3d::Zero};

    /// \brief Field of View for the sensor (camera or lidar).
    private: double fov{1.047}; // Default FOV (approx 60 degrees)

    /// \brief The name of the parent entity (model or link) this sensor is attached to.
    private: std::string parentEntityName;
  };

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
          << "], FOV: [" << this->fov << "]" << std::endl;
    return true;
  }

  //////////////////////////////////////////////////
  bool RtSensor::Update(const std::chrono::steady_clock::duration &_now)
  {
    // For RtSensor, the actual rendering and data generation is handled by the
    // WGPURtSensor system plugin. This method is primarily here to fulfill the
    // base class requirement and can be used for any internal sensor state updates.
    // We can add more specific update logic here if needed for the sensor itself,
    // independent of the WGPU rendering.
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

} // namespace rtsensor


GZ_ADD_PLUGIN(rtsensor::RtSensor, gz::sensors::Sensor)


GZ_ADD_PLUGIN_ALIAS(rtsensor::RtSensor, "rtsensor::RtSensor")
