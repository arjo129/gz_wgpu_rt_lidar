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
use glam::{Affine3A, Mat3A, Quat, Vec3, Vec3A};
use ndarray::ShapeBuilder;
use std::time::Instant;
use wgpu_rt_lidar::wgpu::{Device, Queue};
use wgpu_rt_lidar::{lidar::Lidar, utils::get_raytracing_gpu, vertex, AssetMesh, Instance, Vertex};

#[no_mangle]
pub extern "C" fn create_mesh() -> *mut Mesh {
    Box::into_raw(Box::new(Mesh {
        vertices: Vec::new(),
        faces: Vec::new(),
    }))
}

#[no_mangle]
pub extern "C" fn free_mesh(ptr: *mut Mesh) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[no_mangle]
pub extern "C" fn add_mesh_vertex(mesh: *mut Mesh, x: f32, y: f32, z: f32) {
    let mut mesh = unsafe {
        assert!(!mesh.is_null());
        &mut *mesh
    };
    mesh.vertices.push(vertex([x, y, z]));
}

#[no_mangle]
pub extern "C" fn add_mesh_face(mesh: *mut Mesh, face: u16) {
    let mut mesh = unsafe {
        assert!(!mesh.is_null());
        &mut *mesh
    };
    mesh.faces.push(face);
}

#[repr(C)]
#[derive(Clone)]
pub struct Mesh {
    vertices: Vec<Vertex>,
    faces: Vec<u16>,
}

#[repr(C)]
pub struct InstanceWrapper {
    instance: Instance,
}

#[no_mangle]
pub extern "C" fn create_instance_wrapper(
    asset_index: usize,
    x: f32,
    y: f32,
    z: f32,
    qx: f32,
    qy: f32,
    qz: f32,
    qw: f32,
) -> *mut InstanceWrapper {
    Box::into_raw(Box::new(InstanceWrapper {
        instance: Instance {
            asset_mesh_index: asset_index,
            transform: Affine3A {
                matrix3: Mat3A::from_quat(Quat::from_xyzw(qx, qy, qz, qw)),
                translation: Vec3A::new(x, y, z),
            },
        },
    }))
}

#[no_mangle]
pub extern "C" fn free_instance_wrapper(ptr: *mut InstanceWrapper) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[repr(C)]
pub struct RtSceneBuilder {
    meshes: Vec<Mesh>,
    instances: Vec<Instance>,
}

#[no_mangle]
pub extern "C" fn create_rt_scene_builder() -> *mut RtSceneBuilder {
    Box::into_raw(Box::new(RtSceneBuilder {
        meshes: Vec::new(),
        instances: Vec::new(),
    }))
}

#[no_mangle]
pub extern "C" fn free_rt_scene_builder(ptr: *mut RtSceneBuilder) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[no_mangle]
pub extern "C" fn add_mesh(ptr: *mut RtSceneBuilder, mesh: &Mesh) -> usize {
    let mut builder = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    let index = builder.meshes.len();
    builder.meshes.push(mesh.clone());
    return index;
}

#[no_mangle]
pub extern "C" fn add_instance(ptr: *mut RtSceneBuilder, instance: *mut InstanceWrapper) -> usize {
    let mut builder = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let mut instance = unsafe {
        assert!(!instance.is_null());
        &mut *instance
    };

    let id = builder.instances.len();
    builder.instances.push(instance.instance.clone());
    id
}

#[repr(C)]
pub struct RtRuntime {
    device: Device,
    queue: Queue,
}

impl RtRuntime {
    pub async fn new() -> Self {
        let instance = wgpu_rt_lidar::wgpu::Instance::default();
        let (_, device, queue) = get_raytracing_gpu(&instance).await;
        RtRuntime { device, queue }
    }
}
#[no_mangle]
pub extern "C" fn create_rt_runtime() -> *mut RtRuntime {
    Box::into_raw(Box::new(futures::executor::block_on(RtRuntime::new())))
}

#[no_mangle]
pub extern "C" fn free_rt_runtime(ptr: *mut RtRuntime) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[repr(C)]
pub struct RtSceneUpdate {
    pub updates: Vec<Instance>,
    pub indices: Vec<usize>,
}

#[no_mangle]
pub extern "C" fn create_rt_scene_update() -> *mut RtSceneUpdate {
    Box::into_raw(Box::new(RtSceneUpdate {
        updates: Vec::new(),
        indices: Vec::new(),
    }))
}

#[no_mangle]
pub extern "C" fn add_update(
    ptr: *mut RtSceneUpdate,
    instance_wrapper: *mut InstanceWrapper,
    index: usize,
) {
    let mut update = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    let instance = unsafe {
        assert!(!instance_wrapper.is_null());
        &mut *instance_wrapper
    };
    update.updates.push(instance.instance.clone());
    update.indices.push(index);
}

#[no_mangle]
pub extern "C" fn free_rt_scene_update(ptr: *mut RtSceneUpdate) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[repr(C)]
pub struct RtScene {
    scene: wgpu_rt_lidar::RayTraceScene,
    //rec: rerun::RecordingStream
}

#[no_mangle]
pub extern "C" fn create_rt_scene(
    runtime: *mut RtRuntime,
    builder: *mut RtSceneBuilder,
) -> *mut RtScene {
    let runtime = unsafe {
        assert!(!runtime.is_null());
        &mut *runtime
    };

    let builder = unsafe {
        assert!(!builder.is_null());
        &mut *builder
    };

    //let rec = rerun::RecordingStreamBuilder::new("debug_viz")
    //  .spawn()
    //  .unwrap();
    let scene = futures::executor::block_on(wgpu_rt_lidar::RayTraceScene::new(
        &runtime.device,
        &runtime.queue,
        &builder
            .meshes
            .iter()
            .map(|m| AssetMesh {
                vertex_buf: m.vertices.clone(),
                index_buf: m.faces.clone(),
            })
            .collect::<Vec<AssetMesh>>(),
        &builder.instances,
    ));
    Box::into_raw(Box::new(RtScene {
        scene,
        //rec
    }))
}

#[no_mangle]
pub extern "C" fn set_transforms(
    ptr: *mut RtScene,
    device: *mut RtRuntime,
    updates: *mut RtSceneUpdate,
) {
    let scene = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    let device = unsafe {
        assert!(!device.is_null());
        &mut *device
    };
    let updates = unsafe {
        assert!(!updates.is_null());
        &mut *updates
    };
    //println!("Setting transforms {:?}", updates.updates);
    futures::executor::block_on(scene.scene.set_transform(
        &device.device,
        &device.queue,
        &updates.updates,
        &updates.indices,
    ));

    //scene.scene.visualize(&scene.rec);
}

#[no_mangle]
pub extern "C" fn free_rt_scene(ptr: *mut RtScene) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[repr(C)]
pub struct ViewMatrix {
    pub view: glam::Mat4,
}

#[no_mangle]
pub extern "C" fn create_view_matrix(
    x: f32,
    y: f32,
    z: f32,
    qx: f32,
    qy: f32,
    qz: f32,
) -> *mut ViewMatrix {
    Box::into_raw(Box::new(ViewMatrix {
        view: glam::Mat4::from_translation(glam::Vec3::new(x, y, z))
            * glam::Mat4::from_quat(glam::Quat::from_xyzw(qx, qy, qz, 1.0)),
    }))
}

#[no_mangle]
pub extern "C" fn free_view_matrix(ptr: *mut ViewMatrix) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[repr(C)]
pub struct RtDepthCamera {
    camera: wgpu_rt_lidar::depth_camera::DepthCamera,
}

#[no_mangle]
pub extern "C" fn create_rt_depth_camera(
    runtime: *mut RtRuntime,
    width: u32,
    height: u32,
    fov: f32,
) -> *mut RtDepthCamera {
    let runtime = unsafe {
        assert!(!runtime.is_null());
        &mut *runtime
    };

    let camera =
        wgpu_rt_lidar::depth_camera::DepthCamera::new(&runtime.device, width, height, fov, 50.0);
    Box::into_raw(Box::new(RtDepthCamera {
        camera: futures::executor::block_on(camera),
    }))
}

#[repr(C)]
pub struct ImageData {
    pub ptr: *mut u16,
    pub len: usize,
    pub width: u32,
    pub height: u32,
}

#[no_mangle]
pub extern "C" fn render_depth(
    ptr: *mut RtDepthCamera,
    scene: *mut RtScene,
    runtime: *mut RtRuntime,
    view: *mut ViewMatrix,
) -> ImageData {
    let camera = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let scene = unsafe {
        assert!(!scene.is_null());
        &mut *scene
    };

    let runtime = unsafe {
        assert!(!runtime.is_null());
        &mut *runtime
    };

    let view = unsafe {
        assert!(!view.is_null());
        &mut *view
    };
    let start_time = Instant::now();

    //scene.scene.visualize(&scene.rec);
    let res = futures::executor::block_on(camera.camera.render_depth_camera(
        &scene.scene,
        &runtime.device,
        &runtime.queue,
        view.view.inverse(),
    ));
    let width = camera.camera.width() as u32;
    let height = camera.camera.height() as u32;
    let converted_data: Vec<u16> = res.iter().map(|x| (x * 1000.0) as u16).collect();

    let mut boxed_data = converted_data.into_boxed_slice();
    let data_ptr = boxed_data.as_mut_ptr();
    let data_len = boxed_data.len();
    std::mem::forget(boxed_data); // Prevent Rust from deallocating the memory

    let elapsed2 = start_time.elapsed();
    //println!("Render time for CAMERA: {:.2}ms", elapsed2.as_secs_f64() * 1000.0);

    // Return the image data struct
    ImageData {
        ptr: data_ptr,
        len: data_len,
        width,
        height,
    }
}

#[no_mangle]
pub extern "C" fn free_image_data(image_data: ImageData) {
    if image_data.ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(std::slice::from_raw_parts_mut(
            image_data.ptr,
            image_data.len,
        )));
    }
}

#[no_mangle]
pub extern "C" fn free_rt_depth_camera(ptr: *mut RtDepthCamera) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

struct Rt3DLidarConfiguration {
    num_lasers: usize,
    min_vertical_angle: f32,
    max_vertical_angle: f32,
    step_vertical_angle: f32,
    min_horizontal_angle: f32,
    max_horizontal_angle: f32,
    step_horizontal_angle: f32,
    num_steps: usize,
}

#[no_mangle]
fn new_lidar_config(
    num_lasers: usize,
    num_steps: usize,
    min_vertical_angle: f32,
    max_vertical_angle: f32,
    step_vertical_angle: f32,
    min_horizontal_angle: f32,
    max_horizontal_angle: f32,
    step_horizontal_angle: f32,
) -> *mut Rt3DLidarConfiguration {
    Box::into_raw(Box::new(Rt3DLidarConfiguration {
        num_lasers,
        min_vertical_angle,
        max_vertical_angle,
        step_vertical_angle,
        min_horizontal_angle,
        max_horizontal_angle,
        step_horizontal_angle,
        num_steps: num_steps,
    }))
}

#[no_mangle]
fn free_lidar_config(ptr: *mut Rt3DLidarConfiguration) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

impl Rt3DLidarConfiguration {
    fn to_individual_beam_directions(&self) -> Vec<glam::Vec3> {
        let mut directions = Vec::new();
        for i in 0..self.num_lasers {
            let vertical_angle = self.min_vertical_angle + i as f32 * self.step_vertical_angle;
            for j in 0..self.num_steps {
                let horizontal_angle =
                    self.min_horizontal_angle + j as f32 * self.step_horizontal_angle;
                let direction = glam::Vec3::new(
                    vertical_angle.cos() * horizontal_angle.cos(),
                    vertical_angle.cos() * horizontal_angle.sin(),
                    vertical_angle.sin(),
                );
                directions.push(direction);
                //println!("length: {}", direction.length());
            }
        }
        directions
    }
}

#[repr(C)]
struct RtPointCloud {
    points: *mut f32,
    length: usize,
}

#[no_mangle]
fn free_pointcloud(ptr: *mut RtPointCloud) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        let point_cloud = &*ptr;
        // Free the points array
        if !point_cloud.points.is_null() {
            let _ = Vec::from_raw_parts(point_cloud.points, point_cloud.length, point_cloud.length);
        }
    }
}

#[repr(C)]
struct RtLidar {
    lidar: wgpu_rt_lidar::lidar::Lidar,
}

#[no_mangle]
pub extern "C" fn create_rt_lidar(
    runtime: *mut RtRuntime,
    lidar_config: *mut Rt3DLidarConfiguration,
) -> *mut RtLidar {
    let runtime = unsafe {
        assert!(!runtime.is_null());
        &mut *runtime
    };

    let lidar_config = unsafe {
        assert!(!lidar_config.is_null());
        &mut *lidar_config
    };

    let beams = lidar_config.to_individual_beam_directions();

    let lidar = wgpu_rt_lidar::lidar::Lidar::new(&runtime.device, beams);
    Box::into_raw(Box::new(RtLidar {
        lidar: futures::executor::block_on(lidar),
    }))
}

#[no_mangle]
pub extern "C" fn render_lidar(
    ptr: *mut RtLidar,
    scene: *mut RtScene,
    runtime: *mut RtRuntime,
    view: *mut ViewMatrix,
) -> RtPointCloud {
    let lidar = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let scene = unsafe {
        assert!(!scene.is_null());
        &mut *scene
    };

    let runtime = unsafe {
        assert!(!runtime.is_null());
        &mut *runtime
    };

    let view = unsafe {
        assert!(!view.is_null());
        &mut *view
    };

    let lidar_pose = Affine3A::from_mat4(view.view);

    let start_time = Instant::now();
    //scene.scene.visualize(&scene.rec);
    let mut res = futures::executor::block_on(lidar.lidar.render_lidar_pointcloud(
        &scene.scene,
        &runtime.device,
        &runtime.queue,
        &lidar_pose,
    ));
    let elapsed = start_time.elapsed();
    //println!("Render time for LiDAR: {:.2}ms", elapsed.as_secs_f64() * 1000.0);
    //println!("Number of points rendered: {}", res.len());
    //println!("Points: {:?}", res);
    let p = res
        .chunks(4)
        .filter(|p| p[3] < Lidar::no_hit_const())
        .map(|p| {
            lidar_pose
                .transform_point3(Vec3::new(p[0], p[1], p[2]))
                .to_array()
        });
    //lidar.lidar.visualize_rays(&scene.rec, &lidar_pose, "lidar_beams");
    //scene.rec.log("points", &rerun::Points3D::new(p)).unwrap();

    let mut raw_points: Vec<f32> = res
        .chunks(4)
        .filter(|p| p[3] < Lidar::no_hit_const())
        .flatten()
        .map(|p| *p)
        .collect();

    let point_cloud = RtPointCloud {
        points: raw_points.as_mut_ptr(),
        length: raw_points.len(),
    };
    //println!("Render time for LiDAR: {:.2}ms", elapsed.as_secs_f64() * 1000.0);
    // Prevent the vector from being deallocated
    std::mem::forget(raw_points);

    point_cloud
}

#[no_mangle]
pub extern "C" fn free_rt_lidar(ptr: *mut RtLidar) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}
