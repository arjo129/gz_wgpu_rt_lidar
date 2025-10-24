#pragma once

#include <gz/sensors/Sensor.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/Types.hh>
#include <sdf/Sensor.hh>

#include <string>
#include <chrono>
#include <cmath>

namespace rtsensor
{
  /// \brief Simple abstract RT sensor that can be configured as camera or lidar.
  /// This class defines the properties and basic behavior of a custom raytracing sensor.
  class RtSensor: public gz::sensors::Sensor
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
public: bool Load(const sdf::Sensor & _sdf) override;

    /// \brief Update the sensor and generate data.
public: bool Update(
      const std::chrono::steady_clock::duration & _now) override;

    /// \brief Set the current pose of the sensor in world coordinates.
public: void SetPose(const gz::math::Pose3d & _pose);

    /// \brief Get the latest world pose of the sensor.
public: const gz::math::Pose3d & Pose() const;

    /// \brief Get the sensor type.
public: SensorType Type() const;

    /// \brief Configure the sensor for lidar-specific parameters.
private: void LidarConfig(const sdf::ElementPtr & _sdf);

    /// \brief Configure the sensor for camera-specific parameters.
private: void CameraConfig(const sdf::ElementPtr & _sdf);

    /// \brief Get the topic name for this sensor.
public: const std::string & TopicName() const;

    /// \brief Set the name of the parent entity (e.g., link or model) this sensor is attached to.
public: void SetParentEntity(const gz::sim::Entity & _parentEntity);

    /// \brief Get the name of the parent entity.
public: gz::sim::Entity ParentEntity() const;

    /// \brief Te parent entity (model or link) this sensor is attached to.
private: gz::sim::Entity parentEntity {gz::sim::kNullEntity};

    /// \brief Sensor type (camera or lidar).
private: SensorType sensorType {SensorType::CAMERA};

    /// \brief Current sensor pose in world coordinates.
private: gz::math::Pose3d currentPose {gz::math::Pose3d::Zero};

    /// \brief The last update time of the sensor.
public: std::chrono::steady_clock::duration lastUpdateTime {0};

    /// \brief Configuration parameters for the sensor.
public: struct Config
    {
      std::string topic;
      double updateRate {20.0};
      bool alwaysOn {true};
      bool visualize {false};

      // LiDAR specific
      struct
      {
        size_t num_lasers {16};
        size_t num_steps {720};
        double min_vertical_angle {-M_PI};
        double max_vertical_angle {M_PI};
        double step_vertical_angle {4.0};
        double min_horizontal_angle {-M_PI};
        double max_horizontal_angle {M_PI};
        double step_horizontal_angle {4.0};
        double min_range {0.1};
        double max_range {100.0};
      } lidar;

      // Camera specific
      struct
      {
        unsigned int width {256};
        unsigned int height {256};
        double horizontalFov {60.0};
        double verticalFov {60.0};
        double nearClip {0.1};
        double farClip {100.0};
      } camera;

      // Noise configuration
      struct
      {
        std::string type {"gaussian"};
        double mean {0.0};
        double stddev {0.01};
      } noise;
    } config;
  };
}
