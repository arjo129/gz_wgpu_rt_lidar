#include <gz/plugin/Register.hh>

#include <gz/msgs/entity_factory.pb.h>


#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include <gz/transport/Node.hh>

#include <gz/common/StringUtils.hh>

#include <sdf/Box.hh>
#include <sdf/Plane.hh>

#include "rust_binding.h"
namespace wgpu_sensor { 
  /// \brief A plugin that validates target identification reports.
  class WGPURtSensor :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
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

    // Documentation inherited
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
              const gz::sim::EntityComponentManager &_ecm) override;

    RtScene* rt_scene {nullptr};
    RtRuntime* rt_runtime;
    RtDepthCamera* rt_depth_camera{nullptr};
    std::unordered_map<size_t, size_t> gz_entity_to_rt_instance;
    gz::sim::Entity camera_entity;
    std::chrono::steady_clock::duration last_time;
  };


  WGPURtSensor::WGPURtSensor() {}

  WGPURtSensor::~WGPURtSensor() {
    if (rt_scene != nullptr) {
      free_rt_scene(rt_scene);
    }
    if (rt_depth_camera != nullptr) {
      free_rt_depth_camera(rt_depth_camera);
    }
    free_rt_runtime(rt_runtime);
  }

  Mesh* convertSDFModelToWGPU(const sdf::Geometry& geom) {

     if (geom.Type() == sdf::GeometryType::BOX) {
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
      for (auto x: indices) {
        add_mesh_face(mesh, x);
      }
      return mesh;
    }

  	else if (geom.Type() == sdf::GeometryType::PLANE) {
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
    for (auto x : indices) {
      add_mesh_face(mesh, x);
    }

    return mesh;
  }

    return nullptr;
  }

  void WGPURtSensor::Configure(const gz::sim::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               gz::sim::EntityComponentManager &_ecm,
                               gz::sim::EventManager &_eventMgr) {
    gzmsg << "WGPURtSensor::Configure" << std::endl;
    this->rt_runtime = create_rt_runtime();
    this->rt_depth_camera = create_rt_depth_camera(this->rt_runtime, 256, 256, 59.0);
    this->camera_entity = _entity;
  }

  void WGPURtSensor::PostUpdate(const gz::sim::UpdateInfo &_info,
                               const gz::sim::EntityComponentManager &_ecm) {
    using std::chrono::high_resolution_clock;
    if (_info.paused) {
      return;
    }

    if ((_info.simTime - last_time) < std::chrono::milliseconds(30) && _info.iterations != 0) {
      return;
    }
    last_time = _info.simTime;

    if( this->rt_scene == nullptr) {
      auto rt_scene_builder = create_rt_scene_builder();
       _ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>
              ([this, &rt_scene_builder, &_ecm](auto& entity, auto&& visual, auto&& geometry) {
                  auto geom = geometry->Data();
                  auto model = convertSDFModelToWGPU(geom);
                  if (model != nullptr)
                  {
                    auto mesh_id = add_mesh(rt_scene_builder, model);
                    auto pose = _ecm.Component<gz::sim::components::Pose>(entity);
                    if (!pose) {
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
                  else {
                    gzerr << "Model geometry for" << entity << "  not supported" << std::endl;
                  }
                  return true;
              });
      gzerr << "Creating scene" << std::endl;
      this->rt_scene = create_rt_scene(this->rt_runtime, rt_scene_builder);
      free_rt_scene_builder(rt_scene_builder);
    }
    else {
      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      auto rt_scene_update = create_rt_scene_update();
      _ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>
            ([this, &rt_scene_update, &_ecm](auto& entity, auto&& visual, auto&& geometry) {
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

      set_transforms(rt_scene, rt_runtime, rt_scene_update);

      auto camera_pose =  gz::sim::worldPose(camera_entity, _ecm);
      auto view_matrix = create_view_matrix(camera_pose.Pos().X(), camera_pose.Pos().Y(), camera_pose.Pos().Z(), camera_pose.Rot().X(), camera_pose.Rot().Y(), camera_pose.Rot().Z(), camera_pose.Rot().W());
      render_depth(rt_depth_camera, rt_scene, rt_runtime, view_matrix);
      free_rt_scene_update(rt_scene_update);
      free_view_matrix(view_matrix);
      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
      gzmsg << "Time taken to render depth: " << duration << std::endl;
    }
    
  }
}

GZ_ADD_PLUGIN(wgpu_sensor::WGPURtSensor,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)