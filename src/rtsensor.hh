#ifndef RT_SENSOR_HH_
#define RT_SENSOR_HH_

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/math/Pose3.hh>
#include <string>

namespace rtsensor
{
  /// \brief Simple abstract RT sensor that can be configured as camera or lidar
  /// This is a basic starting point for custom sensor development
  class RtSensor : public gz::sensors::Sensor
  {
    /// \brief Sensor type enumeration
    public: enum class SensorType
    {
      CAMERA,
      LIDAR
    };

    /// \brief Constructor
    public: RtSensor();

    /// \brief Destructor
    public: virtual ~RtSensor();

    /// \brief Load the sensor with SDF parameters
    /// \param[in] _sdf SDF Sensor parameters
    /// \return True if loading was successful
    public: virtual bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Update the sensor and generate data
    /// \param[in] _now The current time
    /// \return True if the update was successful
    public: virtual bool Update(
      const std::chrono::steady_clock::duration &_now) override;

    /// \brief Set the current pose of the sensor in world coordinates
    /// \param[in] _pose Current pose in world coordinates
    public: void SetPose(const gz::math::Pose3d &_pose);

    /// \brief Get the latest world pose of the sensor
    /// \return The latest pose of the sensor
    public: const gz::math::Pose3d &Pose() const;

    /// \brief Get the sensor type
    /// \return The configured sensor type
    public: SensorType GetSensorType() const;

    /// \brief Parse and print LiDAR configuration from SDF
    /// \param[in] _sdf SDF element containing lidar configuration
    private: void ParseLidarConfig(const sdf::ElementPtr &_sdf);

    /// \brief Parse and print camera configuration from SDF
    /// \param[in] _sdf SDF element containing camera configuration
    private: void ParseCameraConfig(const sdf::ElementPtr &_sdf);

    /// \brief Print configuration parameters to console
    private: void PrintConfiguration();

    /// \brief Sensor type (camera or lidar)
    private: SensorType sensorType{SensorType::CAMERA};

    /// \brief Current sensor pose in world coordinates
    private: gz::math::Pose3d currentPose{gz::math::Pose3d::Zero};

    /// \brief Basic configuration parameters
    private: struct Config
    {
      std::string topic;
      double updateRate{20.0};
      bool alwaysOn{true};
      bool visualize{false};

      // LiDAR specific
      struct {
        //lidar parameters
        double fov{1.047};
      } lidar;

      // Camera specific
      struct {
        // Camera parameters
        double fov{1.047};
      } camera;

      // Noise configuration
      struct {
        //noise parameters
      } noise;
      
    } config;
  };
}

#endif // RT_SENSOR_HH_
