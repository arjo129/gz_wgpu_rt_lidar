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
public:
    /// \brief Sensor type enumeration.
    enum class SensorType
    {
      CAMERA,
      LIDAR
    };

    /// \brief Constructor.
    RtSensor();

    /// \brief Destructor.
    virtual ~RtSensor();

    /// \brief Load the sensor with SDF parameters.
    bool Load(const sdf::Sensor & _sdf) override;

    /// \brief Update the sensor and generate data.
    bool Update(
      const std::chrono::steady_clock::duration & _now) override;

    /// \brief Set the current pose of the sensor in world coordinates.
    void SetPose(const gz::math::Pose3d & _pose);

    /// \brief Get the latest world pose of the sensor.
    const gz::math::Pose3d & Pose() const;

    /// \brief Get the sensor type.
    SensorType Type() const;

    /// \brief Get the topic name for this sensor.
    const std::string & TopicName() const;

    /// \brief Set the name of the parent entity (e.g., link or model) this sensor is attached to.
    void SetParentEntity(const gz::sim::Entity & _parentEntity);

    /// \brief Get the name of the parent entity.
    gz::sim::Entity ParentEntity() const;

    /// \brief Configure the sensor for lidar-specific parameters.
    void LidarConfig(const sdf::ElementPtr & _sdf);

    /// \brief Configure the sensor for camera-specific parameters.
    void CameraConfig(const sdf::ElementPtr & _sdf);

    /// \brief Te parent entity (model or link) this sensor is attached to.
    gz::sim::Entity parentEntity {gz::sim::kNullEntity};

    /// \brief Sensor type (camera or lidar).
    SensorType sensorType {SensorType::CAMERA};

    /// \brief Current sensor pose in world coordinates.
    gz::math::Pose3d currentPose {gz::math::Pose3d::Zero};

    /// \brief The last update time of the sensor.
    std::chrono::steady_clock::duration lastUpdateTime {0};

    /// \brief Configuration parameters for the sensor.
    struct Config
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
