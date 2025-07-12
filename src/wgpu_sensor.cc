#include <gz/plugin/Register.hh>

#include <gz/msgs/entity_factory.pb.h>

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <gz/common/StringUtils.hh>

#include <sdf/Box.hh>
#include <sdf/Plane.hh>
#include <sdf/Sensor.hh>
#include <unordered_map>
#include <memory>
#include <string>

#include "rust_binding.h"

#include "rtsensor.hh"

namespace wgpu_sensor
{
  /// \brief A plugin that validates target identification reports.
  class WGPURtSensor :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
    public: WGPURtSensor();
    /// \brief Destructor
    public: ~WGPURtSensor();

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) override;

    public: void RemoveSensorEntities(const gz::sim::EntityComponentManager &_ecm);

    public: Mesh* convertSDFModelToWGPU(const sdf::Geometry& geom);

    RtScene* rt_scene {nullptr};
    RtRuntime* rt_runtime;
    RtDepthCamera* rt_depth_camera{nullptr};
    std::unordered_map<size_t, size_t> gz_entity_to_rt_instance;
    gz::sim::Entity camera_entity;
    std::chrono::steady_clock::duration last_time;
    std::unordered_map<gz::sim::Entity, std::shared_ptr<rtsensor::RtSensor>> entitySensorMap;

    gz::transport::Node node; // Gazebo Transport Node
    std::unordered_map<gz::sim::Entity, gz::transport::Node::Publisher> image_publishers; // Map to store publishers per sensor
  };

  WGPURtSensor::WGPURtSensor() {}

  WGPURtSensor::~WGPURtSensor()
  {
    if (rt_scene != nullptr)
    {
      free_rt_scene(rt_scene);
    }
    if (rt_depth_camera != nullptr)
    {
      free_rt_depth_camera(rt_depth_camera);
    }
    free_rt_runtime(rt_runtime);
  }

  Mesh* WGPURtSensor::convertSDFModelToWGPU(const sdf::Geometry& geom)
  {
    if (geom.Type() == sdf::GeometryType::BOX)
    {
      auto bmesh = geom.BoxShape();
      auto mesh = create_mesh();

      // TOP
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);

      // BOTTOM
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);

      // RIGHT
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);

      // LEFT
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);

      // FRONT
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);

      // BACK
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);

      uint16_t indices[] = {
        0, 1, 2, 2, 3, 0, // top
        4, 5, 6, 6, 7, 4, // bottom
        8, 9, 10, 10, 11, 8, // right
        12, 13, 14, 14, 15, 12, // left
        16, 17, 18, 18, 19, 16, // front
        20, 21, 22, 22, 23, 20, // back
      };
      for (auto x: indices)
      {
        add_mesh_face(mesh, x);
      }
      return mesh;
    }
    else if (geom.Type() == sdf::GeometryType::PLANE)
    {
      auto pmesh = geom.PlaneShape();
      auto mesh = create_mesh();

      add_mesh_vertex(mesh, -pmesh->Size().X()/2, -pmesh->Size().Y()/2, 0.0f);
      add_mesh_vertex(mesh, pmesh->Size().X()/2, -pmesh->Size().Y()/2, 0.0f);
      add_mesh_vertex(mesh, pmesh->Size().X()/2, pmesh->Size().Y()/2, 0.0f);
      add_mesh_vertex(mesh, -pmesh->Size().X()/2, pmesh->Size().Y()/2, 0.0f);

      uint16_t indices[] = {
        0, 1, 2,
        2, 3, 0
      };
      for (auto x : indices)
      {
        add_mesh_face(mesh, x);
      }

      return mesh;
    }
    else if (geom.Type() == sdf::GeometryType::MESH)
    {
      auto mesh_shape = geom.MeshShape();
      auto mesh = create_mesh();

      if (!mesh_shape)
      {
        gzerr << "Failed to get MeshShape from geometry." << std::endl;
        return nullptr;
      }

      const std::string &uri = mesh_shape->Uri();
      const gz::common::Mesh *mesh_data = gz::common::MeshManager::Instance()->Load(uri);
      if (!mesh_data)
      {
        gzerr << "Failed to load mesh from URI: " << uri << std::endl;
        return nullptr;
      }
      if (!mesh)
      {
        gzerr << "Failed to create_mesh for MESH." << std::endl;
        return nullptr;
      }

      const gz::math::Vector3d scale = mesh_shape->Scale();
      // 0 scale error?

      // uint16_t might overflow
      uint16_t vertex_offset = 0;

      for (unsigned int i = 0; i < mesh_data->SubMeshCount(); ++i)
      {
        auto submesh_weak = mesh_data->SubMeshByIndex(i);
        auto submesh_locked = submesh_weak.lock();
        if (!submesh_locked) continue;
        const gz::common::SubMesh *submesh = submesh_locked.get();

        // Add vertices (scaled)
        for (unsigned int v = 0; v < submesh->VertexCount(); ++v)
        {
          const auto &vertex = submesh->Vertex(v);
          add_mesh_vertex(mesh,
            static_cast<float>(vertex.X() * scale.X()),
            static_cast<float>(vertex.Y() * scale.Y()),
            static_cast<float>(vertex.Z() * scale.Z()));
        }

        // Add indices with offset
        for (unsigned int x = 0; x < submesh->IndexCount(); ++x)
        {
          uint16_t index = submesh->Index(x) + vertex_offset;
          add_mesh_face(mesh, index);
        }

        vertex_offset += submesh->VertexCount();
      }

      return mesh;
    }

    return nullptr;
  }

  void WGPURtSensor::Configure(const gz::sim::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               gz::sim::EntityComponentManager &_ecm,
                               gz::sim::EventManager &_eventMgr)
  {
    gzmsg << "WGPURtSensor::Configure" << std::endl;
    this->rt_runtime = create_rt_runtime();
    this->rt_depth_camera = create_rt_depth_camera(this->rt_runtime, 256, 256, 59.0);
    this->camera_entity = _entity;
  }

  void WGPURtSensor::PreUpdate(const gz::sim::UpdateInfo &_info,
                               gz::sim::EntityComponentManager &_ecm)
  {
    // Detect new custom sensors and create RtSensor instances for them
    _ecm.EachNew<gz::sim::components::CustomSensor,
                 gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::CustomSensor *_custom,
          const gz::sim::components::ParentEntity *_parent) -> bool
      {
        // Get the sensor's scoped name (e.g., "world_name::model_name::link_name::sensor_name")
        // Remove world scope for cleaner names if needed for internal logic
        auto sensorScopedName = gz::sim::removeParentScope(
            gz::sim::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName); // Ensure SDF data has the correct name

        // If the sensor topic is not explicitly set in SDF, generate a default one
        if (data.Topic().empty())
        {
          std::string topic = gz::sim::scopedName(_entity, _ecm) + "/rt_sensor";
          data.SetTopic(topic);
        }

        // Create an instance of our custom RtSensor
        auto sensor = std::make_shared<rtsensor::RtSensor>();
        if (!sensor->Load(data))
        {
          gzerr << "[WGPURtSensor] Failed to load RtSensor for entity ["
                << _entity << "] with name [" << sensorScopedName << "]" << std::endl;
          return false;
        }

        // Set the parent entity name on the RtSensor instance for later pose lookup
        auto parentNameComp = _ecm.Component<gz::sim::components::Name>(_parent->Data());
        if (parentNameComp)
        {
          sensor->SetParentEntityName(parentNameComp->Data());
        }
        else
        {
          gzerr << "[WGPURtSensor] Parent entity name not found for sensor entity [" << _entity << "]" << std::endl;
        }

        this->image_publishers[_entity] = this->node.Advertise<gz::msgs::Image>(sensor->TopicName());
        gzmsg << "[WGPURtSensor] Advertising image topic [" << sensor->TopicName()
              << "] for sensor [" << sensor->Name() << "]" << std::endl;

        // Store the new sensor in our map
        this->entitySensorMap.insert(std::make_pair(_entity, std::move(sensor)));
        gzmsg << "[WGPURtSensor] Registered new custom sensor for entity ["
              << _entity << "] named [" << sensorScopedName << "]" << std::endl;
        return true;
      });
  }


  void WGPURtSensor::PostUpdate(const gz::sim::UpdateInfo &_info,
                                const gz::sim::EntityComponentManager &_ecm)
  {
    using std::chrono::high_resolution_clock;
    if (_info.paused)
    {
      return;
    }

    if ((_info.simTime - last_time) < std::chrono::milliseconds(30) && _info.iterations != 0)
    {
      return;
    }
    last_time = _info.simTime;

    if (this->rt_scene == nullptr)
    {
      auto rt_scene_builder = create_rt_scene_builder();
      _ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>
        ([this, &rt_scene_builder, &_ecm](auto& entity, auto&& visual, auto&& geometry)
        {
          auto geom = geometry->Data();
          auto model = convertSDFModelToWGPU(geom);
          if (model != nullptr)
          {
            auto mesh_id = add_mesh(rt_scene_builder, model);
            auto pose = _ecm.Component<gz::sim::components::Pose>(entity);
            if (!pose)
            {
              gzmsg << "No pose component found for entity " << entity << std::endl;
              return true;
            }

            auto instance = create_instance_wrapper(mesh_id, pose->Data().Pos().X(), pose->Data().Pos().Y(), pose->Data().Pos().Z(), pose->Data().Rot().X(), pose->Data().Rot().Y(), pose->Data().Rot().Z(), pose->Data().Rot().W());
            auto inst_id = add_instance(rt_scene_builder, instance);
            free_instance_wrapper(instance);
            gzerr << "Creating instance " << inst_id << "For entity" <<  entity << "With pose" << pose << std::endl;
            gz_entity_to_rt_instance[entity] = inst_id;
            free_mesh(model);
          }
          else
          {
            gzerr << "Model geometry for" << entity << "  not supported" << std::endl;
          }
          return true;
        });
      gzerr << "Creating scene" << std::endl;
      this->rt_scene = create_rt_scene(this->rt_runtime, rt_scene_builder);
      free_rt_scene_builder(rt_scene_builder);
    }
    else
    {
      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      auto rt_scene_update = create_rt_scene_update();
      _ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>
        ([this, &rt_scene_update, &_ecm](auto& entity, auto&& visual, auto&& geometry)
        {
          auto pose = gz::sim::worldPose(entity, _ecm);
          auto instance_handle = gz_entity_to_rt_instance.find(entity);
          if (instance_handle == gz_entity_to_rt_instance.end())
          {
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

      set_transforms(rt_scene, rt_runtime, rt_scene_update);

      // Old camera rendering code
      /*
      auto camera_pose = gz::sim::worldPose(camera_entity, _ecm);
      auto view_matrix = create_view_matrix(camera_pose.Pos().X(), camera_pose.Pos().Y(), camera_pose.Pos().Z(), camera_pose.Rot().X(), camera_pose.Rot().Y(), camera_pose.Rot().Z(), camera_pose.Rot().W());
      render_depth(rt_depth_camera, rt_scene, rt_runtime, view_matrix);
      free_rt_scene_update(rt_scene_update);
      free_view_matrix(view_matrix);
      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
      gzmsg << "Time taken to render depth: " << duration << std::endl; */

      for (auto const& [entityId, sensor] : this->entitySensorMap)
      {
        auto publisher_it = this->image_publishers.find(entityId);
        // Find the parent entity's pose to get the sensor's world pose
        // (assuming the sensor's pose is relative to its parent link/model)
        gz::sim::Entity parentEntity = gz::sim::kNullEntity;
        _ecm.Each<gz::sim::components::Name>(
            [&](const gz::sim::Entity &ent, const gz::sim::components::Name *name)
            {
                if (name->Data() == sensor->ParentEntityName())
                {
                    parentEntity = ent;
                    return false; // Found, stop iteration
                }
                return true;
            });

        if (parentEntity != gz::sim::kNullEntity)
        {
          // Get the world pose of the parent entity
          auto sensor_world_pose = gz::sim::worldPose(parentEntity, _ecm);
          auto view_matrix = create_view_matrix(
            static_cast<float>(sensor_world_pose.Pos().X()), static_cast<float>(sensor_world_pose.Pos().Y()), static_cast<float>(sensor_world_pose.Pos().Z()),
            static_cast<float>(sensor_world_pose.Rot().X()), static_cast<float>(sensor_world_pose.Rot().Y()), static_cast<float>(sensor_world_pose.Rot().Z()), static_cast<float>(sensor_world_pose.Rot().W()));

          ImageData ImageData = render_depth(this->rt_depth_camera, this->rt_scene, this->rt_runtime, view_matrix);
          free_view_matrix(view_matrix);
          gzdbg << "[WGPURtSensor] Rendered from custom sensor [" << sensor->Name() << "]" << std::endl;

          gz::msgs::Image msg;
          msg.set_width(ImageData.width);
          msg.set_height(ImageData.height);
          msg.set_pixel_format_type(gz::msgs::PixelFormatType::L_INT16);
          msg.set_step(ImageData.width * sizeof(uint16_t));
          msg.set_data(ImageData.ptr, ImageData.len * sizeof(uint16_t));

          // Set header timestamp
          *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_info.simTime);
          // Set frame_id to sensor name
          auto frame = msg.mutable_header()->add_data();
          frame->set_key("frame_id");
          frame->add_value(sensor->Name());

          // Publish the message
          publisher_it->second.Publish(msg);

          free_image_data(ImageData);
        }
        else
        {
          gzwarn << "[WGPURtSensor] Could not find parent entity [" << sensor->ParentEntityName()
                 << "] for custom sensor [" << sensor->Name() << "], skipping render." << std::endl;
        }
      }

      free_rt_scene_update(rt_scene_update); // Free the update object
    }

    // --- Update Custom Sensors ---
    if (!_info.paused)
    {
      for (auto &[entity, sensor] : this->entitySensorMap)
      {
        auto baseSensor = std::dynamic_pointer_cast<gz::sensors::Sensor>(sensor);
        if (baseSensor)
        {
          baseSensor->Update(_info.simTime, false); // false to respect update rate
        }
        else
        {
          sensor->Update(_info.simTime);
          gzerr << "[WGPURtSensor] Error casting custom sensor to base Sensor class." << std::endl;
        }
	  }
    }
    // --- Remove Sensors ---
    this->RemoveSensorEntities(_ecm);
  }
  void WGPURtSensor::RemoveSensorEntities(
      const gz::sim::EntityComponentManager &_ecm)
  {

    // Iterate over entities that have been removed and clear them from our map
    _ecm.EachRemoved<gz::sim::components::CustomSensor>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::CustomSensor *) -> bool
      {
        if (this->entitySensorMap.erase(_entity) == 0)
        {
          gzerr << "[WGPURtSensor] Internal error: tried to remove RtSensor for entity ["
                << _entity << "] but it was not found in map." << std::endl;
        }
        else
        {
          gzmsg << "[WGPURtSensor] Removed RtSensor for entity [" << _entity << "]." << std::endl;
        }
        return true;
      });
  }
}

GZ_ADD_PLUGIN(wgpu_sensor::WGPURtSensor,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)
