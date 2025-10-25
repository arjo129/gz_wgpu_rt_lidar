Any contribution that you make to this repository will be under the Apache 2 License, as dictated by that license:
```
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
```   

## Build from source
On ROS2 Jazzy:
```bash
git clone https://github.com/arjo129/gz_wgpu_rt_lidar.git
cd gz_wgpu_rt_lidar
rosdep install --from-paths gz_wgpu_rt_lidar
colcon build
```
Important: Before running Gazebo, ensure your system can find the compiled plugins by sourcing your `install` space:
```bash
source install/setup.bash
```
## Code Quality and Linting
This project uses `ament_uncrustify` to enforce C++ code style. To check your code:
```bash
colcon test --packages-select gz_wgpu_rt_lidar --event-handlers console_direct+
```
The uncrustify configuration can be found in `.uncrustify.cfg` in the root of this package.


## Distrobox usage
I personally love using Distrobox. It might be overkill, but it is great for quickly getting things going. I can spin up specific ROS 2 versions with much ease on a linux system. If you are building in distrobox and have an Nvidia Graphics card, you will need the `--nvidia` flags.


## Rust Analyzer Config
For code completion to work in VS Code you may need to specify the following
```json
{
    "rust-analyzer.linkedProjects": [
        "gz_wgpu_rt_lidar/src/rust_system/Cargo.toml"
    ]
}
```