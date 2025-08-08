#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <unordered_map>

#include "rtsensor.hh"
#include "rust_binding.h"

namespace wgpu_sensor
{
  class RTManager
  {
  public:
    /// \brief Constructor
    RTManager();

    /// \brief Destructor
    ~RTManager();

    /// \brief Initializes the Rust backend runtime
    void Initialize();

    /// \brief Builds the initial ray-tracing scene from world geometry
    void BuildScene(const gz::sim::EntityComponentManager &_ecm);

    /// \brief Updates the transforms of all dynamic objects in the scene
    void UpdateTransforms(const gz::sim::EntityComponentManager &_ecm);

    /// \brief Creates the specific sensor renderer (camera or lidar)
    void CreateSensorRenderer(const gz::sim::Entity &_entity,
                              const std::shared_ptr<rtsensor::RtSensor>& _sensor);

    /// \brief Removes a sensor renderer when the entity is removed
    void RemoveSensorRenderer(const gz::sim::Entity &_entity);

    /// \brief Renders a single sensor and publishes its data
    void RenderSensor(const gz::sim::Entity &_entity,
                      const std::shared_ptr<rtsensor::RtSensor>& _sensor,
                      const gz::sim::UpdateInfo &_info,
                      const gz::sim::EntityComponentManager &_ecm,
                      gz::transport::Node::Publisher &_publisher);

    /// \brief Checks if the scene has been built
    bool IsSceneInitialized() const;

  private:
    /// \brief Converts SDF geometry to a format the Rust backend can use
    Mesh* convertSDFModelToWGPU(const sdf::Geometry& geom);

    /// \brief Pointer to the main Rust runtime
    RtRuntime* rt_runtime{nullptr};

    /// \brief Pointer to the ray-tracing scene on the GPU
    RtScene* rt_scene{nullptr};

    /// \brief Maps a Gazebo entity to a Rust scene instance for pose updates
    std::unordered_map<gz::sim::Entity, size_t> gz_entity_to_rt_instance;

    /// \brief Maps a sensor entity to its specific Rust depth camera renderer
    std::unordered_map<gz::sim::Entity, RtDepthCamera*> rt_depth_cameras;

    /// \brief Maps a sensor entity to its specific Rust LiDAR renderer
    std::unordered_map<gz::sim::Entity, RtLidar*> rt_lidars;
  };
}
