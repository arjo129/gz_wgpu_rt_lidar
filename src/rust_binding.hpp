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

#include <cstdint>
#include <span>
#include <utility>

#include "rust_binding.h"

namespace rust_rt
{

template < typename T, void(*FreeFn)(T*) >
class UniqueHandle
{
public:
  UniqueHandle() = default;
  explicit UniqueHandle(T * _ptr) : ptr(_ptr) {}
  ~UniqueHandle()
  {
    this->Reset();
  }

  UniqueHandle(const UniqueHandle &) = delete;
  UniqueHandle & operator=(const UniqueHandle &) = delete;

  UniqueHandle(UniqueHandle && _other) noexcept : ptr(_other.Release()) {}

  UniqueHandle & operator=(UniqueHandle && _other) noexcept
  {
    if (this != &_other) {
      this->Reset(_other.Release());
    }
    return *this;
  }

  T * Get() const { return this->ptr; }
  explicit operator bool() const { return this->ptr != nullptr; }

  T * Release()
  {
    T * out = this->ptr;
    this->ptr = nullptr;
    return out;
  }

  void Reset(T * _ptr = nullptr)
  {
    if (this->ptr == _ptr) {
      return;
    }
    T * old = std::exchange(this->ptr, _ptr);
    if (old != nullptr) {
      FreeFn(old);
    }
  }

private:
  T * ptr {nullptr};
};

using MeshHandle = UniqueHandle < Mesh, free_mesh >;
using InstanceHandle = UniqueHandle < InstanceWrapper, free_instance_wrapper >;
using SceneBuilderHandle = UniqueHandle < RtSceneBuilder, free_rt_scene_builder >;
using RuntimeHandle = UniqueHandle < RtRuntime, free_rt_runtime >;
using SceneUpdateHandle = UniqueHandle < RtSceneUpdate, free_rt_scene_update >;
using SceneHandle = UniqueHandle < RtScene, free_rt_scene >;
using ViewMatrixHandle = UniqueHandle < ViewMatrix, free_view_matrix >;
using DepthCameraHandle = UniqueHandle < RtDepthCamera, free_rt_depth_camera >;
using LidarConfigHandle = UniqueHandle < Rt3DLidarConfiguration, free_lidar_config >;
using LidarHandle = UniqueHandle < RtLidar, free_rt_lidar >;

class ImageDataOwner
{
public:
  ImageDataOwner() = default;
  explicit ImageDataOwner(ImageData _data) : data(_data) {}

  ImageDataOwner(const ImageDataOwner &) = delete;
  ImageDataOwner & operator=(const ImageDataOwner &) = delete;

  ImageDataOwner(ImageDataOwner && _other) noexcept : data(_other.data)
  {
    _other.data.ptr = nullptr;
    _other.data.len = 0;
    _other.data.width = 0;
    _other.data.height = 0;
  }

  ImageDataOwner & operator=(ImageDataOwner && _other) noexcept
  {
    if (this != &_other) {
      this->Reset();
      this->data = _other.data;
      _other.data.ptr = nullptr;
      _other.data.len = 0;
      _other.data.width = 0;
      _other.data.height = 0;
    }
    return *this;
  }

  ~ImageDataOwner()
  {
    this->Reset();
  }

  void Reset()
  {
    if (this->data.ptr != nullptr) {
      free_image_data(this->data);
    }
    this->data.ptr = nullptr;
    this->data.len = 0;
    this->data.width = 0;
    this->data.height = 0;
  }

  uint32_t Width() const { return this->data.width; }
  uint32_t Height() const { return this->data.height; }

  std::span < const uint16_t > Pixels() const
  {
    return std::span < const uint16_t > (this->data.ptr, this->data.len);
  }

private:
  ImageData data {};
};

class PointCloudOwner
{
public:
  PointCloudOwner() = default;
  explicit PointCloudOwner(RtPointCloud _cloud) : cloud(_cloud) {}

  PointCloudOwner(const PointCloudOwner &) = delete;
  PointCloudOwner & operator=(const PointCloudOwner &) = delete;

  PointCloudOwner(PointCloudOwner && _other) noexcept : cloud(_other.cloud)
  {
    _other.cloud.points = nullptr;
    _other.cloud.length = 0;
  }

  PointCloudOwner & operator=(PointCloudOwner && _other) noexcept
  {
    if (this != &_other) {
      this->Reset();
      this->cloud = _other.cloud;
      _other.cloud.points = nullptr;
      _other.cloud.length = 0;
    }
    return *this;
  }

  ~PointCloudOwner()
  {
    this->Reset();
  }

  void Reset()
  {
    if (this->cloud.points != nullptr) {
      free_pointcloud(&this->cloud);
      this->cloud.points = nullptr;
      this->cloud.length = 0;
    }
  }

  std::span < const float > Points() const
  {
    return std::span < const float > (this->cloud.points, this->cloud.length);
  }

private:
  RtPointCloud cloud {};
};

class MeshBuilder
{
public:
  MeshBuilder() : mesh(create_mesh()) {}

  Mesh * Get() const { return this->mesh.Get(); }

  void AddVertex(float _x, float _y, float _z)
  {
    add_mesh_vertex(this->mesh.Get(), _x, _y, _z);
  }

  void AddFace(uint16_t _idx)
  {
    add_mesh_face(this->mesh.Get(), _idx);
  }

  MeshHandle Release() { return std::move(this->mesh); }

private:
  MeshHandle mesh;
};

class SceneBuilder
{
public:
  SceneBuilder() : builder(create_rt_scene_builder()) {}

  RtSceneBuilder * Get() const { return this->builder.Get(); }

  size_t AddMesh(const MeshHandle & _mesh)
  {
    return add_mesh(this->builder.Get(), _mesh.Get());
  }

  size_t AddMesh(const MeshBuilder & _mesh)
  {
    return add_mesh(this->builder.Get(), _mesh.Get());
  }

  size_t AddInstance(const InstanceHandle & _inst)
  {
    return add_instance(this->builder.Get(), _inst.Get());
  }

  size_t AddInstance(
    size_t _index, float _x, float _y, float _z,
    float _qx, float _qy, float _qz, float _qw)
  {
    auto inst = MakeInstance(_index, _x, _y, _z, _qx, _qy, _qz, _qw);
    // Safe: add_instance copies the instance data into the scene builder.
    return add_instance(this->builder.Get(), inst.Get());
  }

  SceneBuilderHandle Release() { return std::move(this->builder); }

private:
  SceneBuilderHandle builder;
};

class Runtime
{
public:
  Runtime() : runtime(create_rt_runtime()) {}

  RtRuntime * Get() const { return this->runtime.Get(); }

private:
  RuntimeHandle runtime;
};

class Scene
{
public:
  Scene(const Runtime & _runtime, const SceneBuilder & _builder)
  : scene(create_rt_scene(_runtime.Get(), _builder.Get())) {}

  RtScene * Get() const { return this->scene.Get(); }

private:
  SceneHandle scene;
};

class ViewMatrix
{
public:
  ViewMatrix(float _x, float _y, float _z, float _qx, float _qy, float _qz, float _qw)
  : view(create_view_matrix(_x, _y, _z, _qx, _qy, _qz, _qw)) {}

  ViewMatrixHandle Release() { return std::move(this->view); }

  ::ViewMatrix * Get() const { return this->view.Get(); }

private:
  ViewMatrixHandle view;
};

class DepthCamera
{
public:
  DepthCamera(const Runtime & _runtime, uint32_t _width, uint32_t _height, float _fov)
  : camera(create_rt_depth_camera(_runtime.Get(), _width, _height, _fov)) {}

  RtDepthCamera * Get() const { return this->camera.Get(); }

  ImageDataOwner Render(const Scene & _scene, const ViewMatrix & _view,
    const Runtime & _runtime) const
  {
    return ImageDataOwner(render_depth(
      this->camera.Get(),
      _scene.Get(),
      _runtime.Get(),
      _view.Get()));
  }

private:
  DepthCameraHandle camera;
};

class Lidar
{
public:
  Lidar(const Runtime & _runtime, const LidarConfigHandle & _config)
  : lidar(create_rt_lidar(_runtime.Get(), _config.Get())) {}

  RtLidar * Get() const { return this->lidar.Get(); }

  PointCloudOwner Render(const Scene & _scene, const ViewMatrix & _view,
    const Runtime & _runtime) const
  {
    return PointCloudOwner(render_lidar(
      this->lidar.Get(),
      _scene.Get(),
      _runtime.Get(),
      _view.Get()));
  }

private:
  LidarHandle lidar;
};

inline InstanceHandle MakeInstance(
  size_t _index, float _x, float _y, float _z,
  float _qx, float _qy, float _qz, float _qw)
{
  return InstanceHandle(create_instance_wrapper(
    _index, _x, _y, _z, _qx, _qy, _qz, _qw));
}

inline LidarConfigHandle MakeLidarConfig(
  size_t _numLasers,
  size_t _numSteps,
  float _minVerticalAngle,
  float _maxVerticalAngle,
  float _stepVerticalAngle,
  float _minHorizontalAngle,
  float _maxHorizontalAngle,
  float _stepHorizontalAngle)
{
  return LidarConfigHandle(new_lidar_config(
    _numLasers,
    _numSteps,
    _minVerticalAngle,
    _maxVerticalAngle,
    _stepVerticalAngle,
    _minHorizontalAngle,
    _maxHorizontalAngle,
    _stepHorizontalAngle));
}

inline ViewMatrix MakeView(
  float _x, float _y, float _z, float _qx, float _qy, float _qz, float _qw)
{
  return ViewMatrix(_x, _y, _z, _qx, _qy, _qz, _qw);
}

inline ImageDataOwner RenderDepth(
  const DepthCamera & _camera, const Scene & _scene,
  const Runtime & _runtime, const ViewMatrix & _view)
{
  return _camera.Render(_scene, _view, _runtime);
}

inline PointCloudOwner RenderLidar(
  const Lidar & _lidar, const Scene & _scene,
  const Runtime & _runtime, const ViewMatrix & _view)
{
  return _lidar.Render(_scene, _view, _runtime);
}

}  // namespace rust_rt
