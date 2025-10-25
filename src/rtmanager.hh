/*
 * Copyright (C) 2025 Arjo Chakravarty, Shashank Rao
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#pragma once

#include "./rust_binding.h"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

#include "rtsensor.hh"

#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

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

  /// \brief Render job struct
  struct RenderJob
  {
    /// \brief Sensor Entity ID
    gz::sim::Entity entityId;
    /// \brief RT Sensor parameters
    std::shared_ptr < rtsensor::RtSensor > sensor;
    /// \brief Sensor World Pose
    gz::math::Pose3d sensorWorldPose;
    /// \brief The Update Info
    gz::sim::UpdateInfo updateInfo;
    /// \brief Parent frame ID
    std::string parentFrameId;
  };

  /// \brief Adds a render job to the queue
  void QueueRenderJob(const RenderJob & _job);

  /// \brief Builds the initial ray-tracing scene from world geometry
  void BuildScene(const gz::sim::EntityComponentManager & _ecm);

  /// \brief Updates the transforms of all dynamic objects in the scene
  void UpdateTransforms(const gz::sim::EntityComponentManager & _ecm);

  /// \brief Creates the specific sensor renderer (camera or lidar)
  void CreateSensorRenderer(
    const gz::sim::Entity & _entity,
    const std::shared_ptr < rtsensor::RtSensor > &_sensor);

  /// \brief Removes a sensor renderer when the entity is removed
  void RemoveSensorRenderer(const gz::sim::Entity & _entity);

  /// \brief Checks if the scene has been built
  bool IsSceneInitialized() const;
private:
  /// \brief Converts SDF geometry to a format the Rust backend can use
  Mesh * convertSDFModelToWGPU(const sdf::Geometry & geom);

  /// \brief Pointer to the main Rust runtime
  RtRuntime *rt_runtime {nullptr};

  /// \brief Pointer to the ray-tracing scene on the GPU
  RtScene *rt_scene {nullptr};

  /// \brief Maps a Gazebo entity to a Rust scene instance for pose updates
  std::unordered_map < gz::sim::Entity, size_t > gz_entity_to_rt_instance;

  /// \brief Maps a sensor entity to its specific Rust depth camera renderer
  std::unordered_map < gz::sim::Entity, RtDepthCamera * > rt_depth_cameras;

  /// \brief Maps a sensor entity to its specific Rust LiDAR renderer
  std::unordered_map < gz::sim::Entity, RtLidar * > rt_lidars;

  /// \brief Initializes transport node
  gz::transport::Node node;

  /// \brief Stores transport publishers
  std::unordered_map < gz::sim::Entity, gz::transport::Node::Publisher > publishers;

  /// \brief The main render loop function
  void RenderLoop();

  /// \brief The worker thread
  std::thread workerThread;

  /// \brief Queue holds the render jobs
  std::queue < RenderJob > jobQueue;

  /// \brief Mutex (lock) protects the queue from being accessed by both threads at once
  std::mutex queueMutex;

  /// \brief Signals the worker thread that a new job is ready
  std::condition_variable condition;

  /// \brief Flag tells the thread to stop when we're done
  bool stopThread {false};
};
} // namespace wgpu_sensor
