#include "rtmanager.hh"

#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud.pb.h>

#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>

#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>

#include <gz/sim/Util.hh>

#include <sdf/Box.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>

namespace wgpu_sensor
{

//=============================================================================
  RTManager::RTManager()
  {
    this->workerThread = std::thread(&RTManager::RenderLoop, this);
  }

//=============================================================================
  RTManager::~RTManager()
  {
    // Tell the thread to stop
    {
      std::unique_lock < std::mutex > lock(this->queueMutex);
      this->stopThread = true;
    }
    // Wake up the thread so it can see the stop flag
    this->condition.notify_one();
    // Wait for the thread to finish its work and exit
    if (this->workerThread.joinable()) {
      this->workerThread.join();
    }
    if (this->rt_scene != nullptr) {
      free_rt_scene(this->rt_scene);
    }
    for (auto const &[entityId, rtCameraPtr] : this->rt_depth_cameras) {
      if (rtCameraPtr != nullptr) {
        free_rt_depth_camera(rtCameraPtr);
      }
    }
    this->rt_depth_cameras.clear();

    for (auto const &[entityId, rtLidarPtr] : this->rt_lidars) {
      if (rtLidarPtr != nullptr) {
        free_rt_lidar(rtLidarPtr);
      }
    }
    this->rt_lidars.clear();

    free_rt_runtime(this->rt_runtime);
  }

//=============================================================================
  void RTManager::Initialize()
  {
    this->rt_runtime = create_rt_runtime();
  }

//=============================================================================
  bool RTManager::IsSceneInitialized() const
  {
    return this->rt_scene != nullptr;
  }

//=============================================================================
  void RTManager::RenderLoop()
  {
    while (true) {
      RenderJob job;

      // 1. Wait for a job to become available
      {
        std::unique_lock < std::mutex > lock(this->queueMutex);
        this->condition.wait(lock, [this] {
          return !this->jobQueue.empty() || this->stopThread;
        });

        // If we're stopping and there are no jobs, exit the loop
        if (this->stopThread && this->jobQueue.empty()) {
          return;
        }

        // Get the next job from the queue
        job = this->jobQueue.front();
        this->jobQueue.pop();
      }

      auto pubIt = this->publishers.find(job.entityId);
      if (pubIt == this->publishers.end()) {
        // No publisher for this entity, skip
        continue;
      }

      auto view_matrix = create_view_matrix(
        static_cast < float > (job.sensorWorldPose.Pos().X()),
        static_cast < float > (job.sensorWorldPose.Pos().Y()),
        static_cast < float > (job.sensorWorldPose.Pos().Z()),
        static_cast < float > (job.sensorWorldPose.Rot().X()),
        static_cast < float > (job.sensorWorldPose.Rot().Y()),
        static_cast < float > (job.sensorWorldPose.Rot().Z()),
        static_cast < float > (job.sensorWorldPose.Rot().W())
      );

      if (job.sensor->Type() == rtsensor::RtSensor::SensorType::CAMERA) {
        auto it = this->rt_depth_cameras.find(job.entityId);
        if (it == this->rt_depth_cameras.end() || !it->second) {return;}
        RtDepthCamera *currentRtCamera = it->second;

        ImageData imageData = render_depth(currentRtCamera, this->rt_scene, this->rt_runtime,
                                           view_matrix);

        gz::msgs::Image msg;
        msg.set_width(imageData.width);
        msg.set_height(imageData.height);
        msg.set_pixel_format_type(gz::msgs::PixelFormatType::L_INT16);
        msg.set_step(imageData.width * sizeof(uint16_t));
        msg.set_data(imageData.ptr, imageData.len * sizeof(uint16_t));
        *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(job.updateInfo.simTime);

        auto frame = msg.mutable_header()->add_data();
        frame->set_key("frame_id");
        frame->add_value(job.parentFrameId);

        pubIt->second.Publish(msg);
      } else if (job.sensor->Type() == rtsensor::RtSensor::SensorType::LIDAR) {
        auto it = this->rt_lidars.find(job.entityId);
        if (it == this->rt_lidars.end() || !it->second) {return;}
        RtLidar *currentRtLidar = it->second;

        RtPointCloud pointCloudData = render_lidar(currentRtLidar, this->rt_scene, this->rt_runtime,
                                                   view_matrix);

        gz::msgs::PointCloudPacked msg;
        *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(job.updateInfo.simTime);

        auto frame = msg.mutable_header()->add_data();
        frame->set_key("frame_id");
        frame->add_value(job.parentFrameId);

        msg.set_height(1);
        const uint32_t num_points = pointCloudData.length / 4;
        msg.set_width(num_points);

        auto *f = msg.add_field();
        f->set_name("x");
        f->set_offset(0);
        f->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
        f->set_count(1);

        f = msg.add_field();
        f->set_name("y");
        f->set_offset(4);
        f->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
        f->set_count(1);

        f = msg.add_field();
        f->set_name("z");
        f->set_offset(8);
        f->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
        f->set_count(1);

        f = msg.add_field();
        f->set_name("intensity");
        f->set_offset(12);
        f->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
        f->set_count(1);

        msg.set_point_step(sizeof(float) * 4);
        msg.set_row_step(pointCloudData.length * msg.point_step());
        msg.set_data(pointCloudData.points, pointCloudData.length * sizeof(float));

        pubIt->second.Publish(msg);
      }
      free_view_matrix(view_matrix);
    }
  }

  void RTManager::QueueRenderJob(const RenderJob & _job)
  {
    {
      std::unique_lock < std::mutex > lock(this->queueMutex);
      this->jobQueue.push(_job);
    }
    this->condition.notify_one();
  }

  void RTManager::BuildScene(const gz::sim::EntityComponentManager & _ecm)
  {
    gzmsg << "RTManager: Building ray-tracing scene from world geometry." << std::endl;
    auto rt_scene_builder = create_rt_scene_builder();
    _ecm.Each < gz::sim::components::Visual, gz::sim::components::Geometry > (
      [this, &rt_scene_builder, &_ecm](const gz::sim::Entity & entity, const auto *,
      const auto *geometry)
    {
      auto geom = geometry->Data();
      auto model = this->convertSDFModelToWGPU(geom);
      if (model != nullptr) {
        auto mesh_id = add_mesh(rt_scene_builder, model);
        auto pose = _ecm.Component < gz::sim::components::Pose > (entity);
        if (pose) {
          auto instance = create_instance_wrapper(mesh_id, pose->Data().Pos().X(),
                                                  pose->Data().Pos().Y(), pose->Data().Pos().Z(),
                                                  pose->Data().Rot().X(),
                                                  pose->Data().Rot().Y(), pose->Data().Rot().Z(),
                                                  pose->Data().Rot().W());
          auto inst_id = add_instance(rt_scene_builder, instance);
          free_instance_wrapper(instance);
          this->gz_entity_to_rt_instance[entity] = inst_id;
        }
        free_mesh(model);
      }
      return true;
    });

    this->rt_scene = create_rt_scene(this->rt_runtime, rt_scene_builder);
    free_rt_scene_builder(rt_scene_builder);
    gzmsg << "RTManager: Scene built successfully." << std::endl;
  }

  void RTManager::UpdateTransforms(const gz::sim::EntityComponentManager & _ecm)
  {
    auto rt_scene_update = create_rt_scene_update();
    _ecm.Each < gz::sim::components::Visual, gz::sim::components::Geometry >
    ([this, &rt_scene_update, &_ecm](auto & entity, auto && visual, auto && geometry)
    {
      auto pose = gz::sim::worldPose(entity, _ecm);
      auto instance_handle = gz_entity_to_rt_instance.find(entity);
      if (instance_handle == gz_entity_to_rt_instance.end()) {
        gzwarn << "No instance found for entity " << entity << std::endl;
        return true;
      }
      auto instance =
      create_instance_wrapper(instance_handle->second,
                                pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(),
                                pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());

      add_update(rt_scene_update, instance, instance_handle->second);
      free_instance_wrapper(instance);
      return true;
    });

    set_transforms(this->rt_scene, this->rt_runtime, rt_scene_update);
    free_rt_scene_update(rt_scene_update);
  }

  void RTManager::CreateSensorRenderer(
    const gz::sim::Entity & _entity,
    const std::shared_ptr < rtsensor::RtSensor > &_sensor)
  {
    if (_sensor->Type() == rtsensor::RtSensor::SensorType::CAMERA) {
      this->rt_depth_cameras[_entity] = create_rt_depth_camera(
        this->rt_runtime,
        _sensor->config.camera.width,
        _sensor->config.camera.height,
        static_cast < float > (_sensor->config.camera.horizontalFov)
        // ADD: verticalFOV, nearClip, far Clip
      );
    } else if (_sensor->Type() == rtsensor::RtSensor::SensorType::LIDAR) {
      auto lidarConfig = new_lidar_config(
        _sensor->config.lidar.num_lasers,
        _sensor->config.lidar.num_steps,
        static_cast < float > (_sensor->config.lidar.min_vertical_angle),
        static_cast < float > (_sensor->config.lidar.max_vertical_angle),
        static_cast < float > (_sensor->config.lidar.step_vertical_angle),
        static_cast < float > (_sensor->config.lidar.min_horizontal_angle),
        static_cast < float > (_sensor->config.lidar.max_horizontal_angle),
        static_cast < float > (_sensor->config.lidar.step_horizontal_angle)
        // ADD: min_range, max_range
      );
      this->rt_lidars[_entity] = create_rt_lidar(this->rt_runtime, lidarConfig);
      free_lidar_config(lidarConfig);
    }
    if (_sensor->Type() == rtsensor::RtSensor::SensorType::LIDAR) {
      this->publishers[_entity] = this->node.Advertise < gz::msgs::PointCloudPacked >
        (_sensor->TopicName());
    } else if (_sensor->Type() == rtsensor::RtSensor::SensorType::CAMERA) {
      this->publishers[_entity] = this->node.Advertise < gz::msgs::Image > (_sensor->TopicName());
    }
  }

  void RTManager::RemoveSensorRenderer(const gz::sim::Entity & _entity)
  {
    if (this->rt_depth_cameras.count(_entity)) {
      free_rt_depth_camera(this->rt_depth_cameras.at(_entity));
      this->rt_depth_cameras.erase(_entity);
    }
    if (this->rt_lidars.count(_entity)) {
      free_rt_lidar(this->rt_lidars.at(_entity));
      this->rt_lidars.erase(_entity);
    }
  }

  Mesh * RTManager::convertSDFModelToWGPU(const sdf::Geometry & geom)
  {
    if (geom.Type() == sdf::GeometryType::BOX) {
      auto bmesh = geom.BoxShape();
      auto mesh = create_mesh();

      // TOP
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);

      // BOTTOM
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);

      // RIGHT
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);

      // LEFT
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);

      // FRONT
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);

      // BACK
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, -bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);
      add_mesh_vertex(mesh, bmesh->Size().X() / 2, -bmesh->Size().Y() / 2, -bmesh->Size().Z() / 2);

      uint16_t indices[] = {
        0, 1, 2, 2, 3, 0,
        4, 5, 6, 6, 7, 4,
        8, 9, 10, 10, 11, 8,
        12, 13, 14, 14, 15, 12,
        16, 17, 18, 18, 19, 16,
        20, 21, 22, 22, 23, 20,
      };
      for (auto x : indices) {
        add_mesh_face(mesh, x);
      }
      return mesh;
    } else if (geom.Type() == sdf::GeometryType::PLANE) {
      auto pmesh = geom.PlaneShape();
      auto mesh = create_mesh();

      add_mesh_vertex(mesh, -pmesh->Size().X() / 2, -pmesh->Size().Y() / 2, 0.0f);
      add_mesh_vertex(mesh, pmesh->Size().X() / 2, -pmesh->Size().Y() / 2, 0.0f);
      add_mesh_vertex(mesh, pmesh->Size().X() / 2, pmesh->Size().Y() / 2, 0.0f);
      add_mesh_vertex(mesh, -pmesh->Size().X() / 2, pmesh->Size().Y() / 2, 0.0f);

      uint16_t indices[] = {
        0, 1, 2,
        2, 3, 0
      };
      for (auto x : indices) {
        add_mesh_face(mesh, x);
      }
      return mesh;
    } else if (geom.Type() == sdf::GeometryType::MESH) {
      auto mesh_shape = geom.MeshShape();
      auto mesh = create_mesh();
      const std::string & uri = mesh_shape->Uri();
      const gz::common::Mesh *mesh_data = gz::common::MeshManager::Instance()->Load(uri);
      if (!mesh_data) {return nullptr;}

      const gz::math::Vector3d scale = mesh_shape->Scale();
      uint16_t vertex_offset = 0;
      for (unsigned int i = 0; i < mesh_data->SubMeshCount(); ++i) {
        auto submesh_weak = mesh_data->SubMeshByIndex(i);
        auto submesh = submesh_weak.lock();
        if (!submesh) {continue;}
        for (unsigned int v = 0; v < submesh->VertexCount(); ++v) {
          const auto & vertex = submesh->Vertex(v);
          add_mesh_vertex(mesh,
                          static_cast < float > (vertex.X() * scale.X()),
                          static_cast < float > (vertex.Y() * scale.Y()),
                          static_cast < float > (vertex.Z() * scale.Z()));
        }
        for (unsigned int x = 0; x < submesh->IndexCount(); ++x) {
          add_mesh_face(mesh, submesh->Index(x) + vertex_offset);
        }
        vertex_offset += submesh->VertexCount();
      }
      return mesh;
    }
    return nullptr;
  }

} // namespace wgpu_rt_sensor
