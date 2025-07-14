use glam::{Affine3A, Mat3A, Quat, Vec3A};
use ndarray::ShapeBuilder;
use wgpu::Device;
use wgpu_rt_lidar::{utils::get_raytracing_gpu, vertex, AssetMesh, Instance, Vertex};

#[no_mangle]
pub extern "C" fn create_mesh() -> *mut Mesh {
    Box::into_raw(Box::new(Mesh {
        vertices: Vec::new(),
        faces: Vec::new()
    }))
}

#[no_mangle]
pub extern "C" fn free_mesh(ptr: *mut Mesh)
{
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
    faces: Vec<u16>
}

#[repr(C)]
pub struct InstanceWrapper {
    instance: Instance
}


#[no_mangle]
pub extern "C" fn create_instance_wrapper(asset_index: usize, x: f32, y: f32, z:f32, qx:f32, qy:f32, qz:f32, qw:f32) -> *mut InstanceWrapper {
    println!("asset_index: {}, x: {}, y: {}, z: {}, qx: {}, qy: {}, qz: {}, qw: {}", asset_index, x, y, z, qx, qy, qz, qw);
    Box::into_raw(Box::new(InstanceWrapper {
            instance: Instance{
                asset_mesh_index: asset_index,
                transform: Affine3A { matrix3: Mat3A::from_quat(Quat::from_xyzw(qx,qy,qz,qw)), translation: Vec3A::new(x, y, z)}
            }
    }))
}

#[no_mangle]
pub extern "C" fn free_instance_wrapper(ptr: *mut InstanceWrapper){
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
    instances: Vec<Instance>
}

#[no_mangle]
pub extern "C" fn create_rt_scene_builder() -> *mut RtSceneBuilder {
    Box::into_raw(Box::new(RtSceneBuilder {
        meshes: Vec::new(),
        instances: Vec::new()
    }))
}

#[no_mangle]
pub extern "C" fn free_rt_scene_builder(ptr: *mut RtSceneBuilder)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[no_mangle]
pub extern "C" fn add_mesh(ptr: *mut RtSceneBuilder, mesh: &Mesh) -> usize
{
    let mut builder = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    let index = builder.meshes.len();
    builder.meshes.push(mesh.clone());
    return index;
}

#[no_mangle]
pub extern "C" fn add_instance(ptr: *mut RtSceneBuilder, instance: *mut InstanceWrapper) -> usize
{
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
    device: wgpu::Device,
    queue: wgpu::Queue,
}

impl RtRuntime {
    pub async fn new() -> Self {
        let instance = wgpu::Instance::default();
        let (_, device, queue) =  get_raytracing_gpu(&instance).await;

        println!("Found Raytracing enabled GPU");
        RtRuntime {
            device,
            queue
        }
    }
}
#[no_mangle]
pub extern "C" fn create_rt_runtime() -> *mut RtRuntime {
    Box::into_raw(Box::new(futures::executor::block_on(RtRuntime::new())))
}

#[no_mangle]
pub extern "C" fn free_rt_runtime(ptr: *mut RtRuntime)
{
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
    pub indices: Vec<usize>
}

#[no_mangle]
pub extern "C" fn create_rt_scene_update() -> *mut RtSceneUpdate {
    Box::into_raw(Box::new(RtSceneUpdate {
        updates: Vec::new(),
        indices: Vec::new()
    }))
}

#[no_mangle]
pub extern "C" fn add_update(ptr: *mut RtSceneUpdate, instance_wrapper: *mut InstanceWrapper, index: usize) {
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
pub extern "C" fn free_rt_scene_update(ptr: *mut RtSceneUpdate)
{
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
    rec: rerun::RecordingStream
}

#[no_mangle]
pub extern "C" fn create_rt_scene(runtime: *mut RtRuntime, builder: *mut RtSceneBuilder) -> *mut RtScene {
    let runtime = unsafe {
        assert!(!runtime.is_null());
        &mut *runtime
    };

    let builder = unsafe {
        assert!(!builder.is_null());
        &mut *builder
    };

    let scene = 
     futures::executor::block_on(
        wgpu_rt_lidar::RayTraceScene::new(
            &runtime.device, 
            &runtime.queue, 
            &builder.meshes.iter().map(|m| 
                AssetMesh {vertex_buf: m.vertices.clone(), index_buf: m.faces.clone()}).collect::<Vec<AssetMesh>>(),
            &builder.instances));
    Box::into_raw(Box::new(RtScene {
        scene,
        rec: rerun::RecordingStreamBuilder::new("debug_viz").spawn().unwrap()
    }))
}

#[no_mangle]
pub extern "C" fn set_transforms(ptr: *mut RtScene, device: *mut RtRuntime, updates: *mut RtSceneUpdate) {
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
    println!("Setting transforms {:?}", updates.updates);
    futures::executor::block_on(scene.scene.set_transform(&device.device, &device.queue, &updates.updates, &updates.indices));
}

#[no_mangle]
pub extern "C" fn free_rt_scene(ptr: *mut RtScene)
{
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
pub extern "C" fn create_view_matrix(x: f32, y: f32, z: f32, qx:f32, qy:f32, qz:f32) -> *mut ViewMatrix {
    Box::into_raw(Box::new(ViewMatrix {
        view: glam::Mat4::from_translation(glam::Vec3::new(x, y, z)) * glam::Mat4::from_quat(glam::Quat::from_xyzw(qx, qy, qz, 1.0))
    }))
}

#[no_mangle]
pub extern "C" fn free_view_matrix(ptr: *mut ViewMatrix)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[repr(C)]
pub struct RtDepthCamera {
    camera: wgpu_rt_lidar::depth_camera::DepthCamera
}

#[no_mangle]
pub extern "C" fn create_rt_depth_camera(runtime: *mut RtRuntime, width: u32, height: u32, fov: f32) -> *mut RtDepthCamera {

    let runtime = unsafe {
        assert!(!runtime.is_null());
        &mut *runtime
    };

    let camera = wgpu_rt_lidar::depth_camera::DepthCamera::new(&runtime.device, width, height, fov, 50.0);
    Box::into_raw(Box::new(RtDepthCamera {
        camera: futures::executor::block_on(camera)
    }))
}

#[no_mangle]
pub extern "C" fn render_depth(ptr: *mut RtDepthCamera, scene: *mut RtScene, runtime: *mut RtRuntime, view: *mut ViewMatrix) {
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

    scene.scene.visualize(&scene.rec);
    let res = futures::executor::block_on(camera.camera.render_depth_camera(&scene.scene, &runtime.device, &runtime.queue, view.view.inverse()));
    let mut image = ndarray::Array::<u16, _>::from_elem(
        (
            camera.camera.width() as usize,
            camera.camera.height() as usize,
        )
            .f(),
        65535,
    );
    for (i, x) in res.iter().enumerate() {
        let x = (x * 1000.0) as u16;
        image[(
            i / camera.camera.width() as usize,
            i % camera.camera.width() as usize,
        )] = x;
    }
    let depth_image = rerun::DepthImage::try_from(image)
        .unwrap()
        .with_meter(1000.0)
        .with_colormap(rerun::components::Colormap::Viridis);
    scene.rec.log("depth_cloud", &depth_image);

    //println!("{:?}", res.iter().fold(0.0, |acc, x| x.w.max(acc))
    
}

#[no_mangle]
pub extern "C" fn free_rt_depth_camera(ptr: *mut RtDepthCamera)
{
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
    num_steps: usize
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
    step_horizontal_angle: f32
) -> *mut Rt3DLidarConfiguration {
    Box::into_raw(Box::new(Rt3DLidarConfiguration {
        num_lasers,
        min_vertical_angle,
        max_vertical_angle,
        step_vertical_angle,
        min_horizontal_angle,
        max_horizontal_angle,
        step_horizontal_angle,
        num_steps: num_steps
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
                let horizontal_angle = self.min_horizontal_angle + j as f32 * self.step_horizontal_angle;
                let direction = glam::Vec3::new(
                    vertical_angle.cos() * horizontal_angle.sin(),
                    vertical_angle.sin(),
                    vertical_angle.cos() * horizontal_angle.cos()
                );
                directions.push(direction);
            }
        }
        directions
    }
}

#[repr(C)]
struct RtPointCloud
{
    points: *mut f32,
    length: usize
}

#[no_mangle]
fn free_pointcloud(ptr: *mut RtPointCloud) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        let point_cloud = Box::from_raw(ptr);
        // Free the points array
        if !point_cloud.points.is_null() {
            let _ = Vec::from_raw_parts(point_cloud.points, point_cloud.length, point_cloud.length);
        }
        drop(point_cloud);
    }
}

#[repr(C)]
struct RtLidar{
    lidar: wgpu_rt_lidar::lidar::Lidar
}

#[no_mangle]
pub extern "C" fn create_rt_lidar(runtime: *mut RtRuntime, lidar_config: *mut Rt3DLidarConfiguration) -> *mut RtLidar {
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
        lidar: futures::executor::block_on(lidar)
    }))
}

#[no_mangle]
pub extern "C" fn render_lidar(ptr: *mut RtLidar, scene: *mut RtScene, runtime: *mut RtRuntime, view: *mut ViewMatrix) -> RtPointCloud{
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

    let mut res = futures::executor::block_on(lidar.lidar.render_lidar_beams(&scene.scene, &runtime.device, &runtime.queue, &Affine3A::from_mat4(view.view.inverse())));
    

    let point_cloud = RtPointCloud {
        points: res.as_mut_ptr(),
        length: res.len(),
    };

    // Prevent the vector from being deallocated
    std::mem::forget(res);

    point_cloud
}

#[no_mangle]
pub extern "C" fn free_rt_lidar(ptr: *mut RtLidar)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}