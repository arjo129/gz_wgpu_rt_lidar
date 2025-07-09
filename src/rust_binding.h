// This file is part of the Rust binding for the RT Scene library.
#pragma once

#include <stdint.h>

extern "C" {

/// @brief Opaque structure representing a mesh.
struct Mesh;

/// @brief Create a new mesh
/// @return a mesh
Mesh* create_mesh();

/// @brief Free a mesh
/// @param mesh 
void free_mesh(Mesh* mesh);

/// @brief Add a vertex to the mesh
void add_mesh_vertex(Mesh* mesh, float x, float y, float z);

/// @brief Add a normal to the mesh
void add_mesh_face(Mesh* mesh, uint16_t);

/// @brief Opaque structure representing an instance wrapper.
struct InstanceWrapper;

/// @brief Create a new instance wrapper
InstanceWrapper* create_instance_wrapper(size_t index, float x, float y, float z, float qx, float qy, float qz, float qw);

/// @brief Set the mesh of the instance wrapper
void free_instance_wrapper(InstanceWrapper* instance_wrapper);

struct RtSceneBuilder;

RtSceneBuilder* create_rt_scene_builder();

void free_rt_scene_builder(RtSceneBuilder* rt_scene_builder);

size_t add_mesh(RtSceneBuilder* rt_scene_builder, Mesh* mesh);

size_t add_instance(RtSceneBuilder* rt_scene_builder, InstanceWrapper* instance_wrapper);

struct RtRuntime;

RtRuntime* create_rt_runtime();

void free_rt_runtime(RtRuntime* rt_runtime);

struct RtSceneUpdate;

RtSceneUpdate* create_rt_scene_update();

void add_update(RtSceneUpdate* rt_scene_update, InstanceWrapper* wrapper, size_t instance_index);

void free_rt_scene_update(RtSceneUpdate* rt_scene_update);

struct RtScene;

RtScene* create_rt_scene(RtRuntime* rt_runtime, RtSceneBuilder* rt_scene_builder);

void set_transforms(RtScene* rt_scene, RtRuntime* rt_runtime, RtSceneUpdate* rt_scene_update);

void free_rt_scene(RtScene* rt_scene);

struct ViewMatrix;

ViewMatrix* create_view_matrix(float x, float y, float z, float qx, float qy, float qz, float qw);

void free_view_matrix(ViewMatrix* view_matrix);

struct RtDepthCamera;

RtDepthCamera* create_rt_depth_camera(RtRuntime* rt_runtime, size_t width, size_t height, float fov);

void render_depth(RtDepthCamera* rt_depth_camera, RtScene* rt_scene, RtRuntime* rt_runtime, ViewMatrix* view_matrix);

void free_rt_depth_camera(RtDepthCamera* rt_depth_camera);

struct Rt3DLidarConfiguration;

Rt3DLidarConfiguration* new_lidar_config(
    size_t num_lasers,
    size_t num_steps,
    float min_vertical_angle,
    float max_vertical_angle,
    float step_vertical_angle,
    float min_horizontal_angle,
    float max_horizontal_angle,
    float step_horizontal_angle
);

void free_lidar_config(Rt3DLidarConfiguration* config);
}
