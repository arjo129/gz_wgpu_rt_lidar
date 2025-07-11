#pragma once

#include <gz/sensors/Sensor.hh>
#include <gz/math/Pose3.hh>
#include <string>
#include <sdf/Sensor.hh>

namespace rtsensor
{
  /// \brief Simple abstract RT sensor that can be configured as camera or lidar.
  /// This class defines the properties and basic behavior of a custom raytracing sensor.
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
    public: virtual ~RtSensor();

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

    /// \brief Topic name for publishing sensor data.
    private: std::string topic;

    /// \brief Get the topic name for this sensor.
    public: const std::string& TopicName() const;

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
}
