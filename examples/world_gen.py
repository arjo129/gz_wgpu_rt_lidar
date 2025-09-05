import argparse
import math
import sys

MESH_URI = "meshes/hatchback.obj"
MESH_SCALE = "0.0254 0.0254 0.0254"

def world_header():
    return f"""<?xml version="1.0" ?>
<sdf version="1.11">
  <world name="generated_world">

    <plugin filename="wgpu_rt_sensor" name="wgpu_sensor::WGPURtSensor"/>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin
            filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    
    <scene>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
    </scene>

    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
"""

def world_footer():
    return """  </world>
</sdf>
"""

def ground_plane(size):
    return f"""
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{size} {size}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{size} {size}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
            <specular>0.5 0.5 0.5 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""

def hatch_model(i, j, x, y, color):
    rgba = "0 0 1 1" if color == "blue" else "1 0 0 1"
    name = f"hatchback_{i}_{j}_{color}"
    return f"""
    <model name="{name}">
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>{MESH_URI}</uri>
              <scale>{MESH_SCALE}</scale>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>{MESH_URI}</uri>
              <scale>{MESH_SCALE}</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
            <specular>{rgba}</specular>
          </material>
        </visual>
      </link>
    </model>
"""

def box(i, j, x, y, color):
    rgba = "0 0 1 1" if color == "blue" else "1 0 0 1"
    name = f"box_{i}_{j}_{color}"
    return f"""
    <model name="{name}">
      <pose>{x:.3f} {y:.3f} 0.5 0 0 0</pose>
      <static>true</static>
      <link name="box_link">
        <inertial>
          <inertia>
            <ixx>0.16666</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.16666</iyy>
            <iyz>0</iyz>
            <izz>0.16666</izz>
          </inertia>
          <mass>1.0</mass>
        </inertial>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>2 2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
            <specular>{rgba}</specular>
          </material>
        </visual>
      </link>
    </model>
"""

def gazebo_lidar(idx, x, y, z=1.0, topic_prefix="/lidar/gz", steps=360):
    return f"""
    <model name="gazebo_lidar_{idx}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
        <sensor name="lidar" type="gpu_lidar">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <lidar>
            <scan>
              <vertical>
                <samples>16</samples>
                <min_angle>-0.2617</min_angle>
                <max_angle>0.2617</max_angle>
                <resolution>1</resolution>
              </vertical>
              <horizontal>
                <min_angle>-3.1415</min_angle>
                <max_angle>3.1415</max_angle>
                <samples>{steps}</samples>
                <resolution>1</resolution>
              </horizontal>
            </scan>
            <range><min>0.1</min><max>100.0</max></range>
            <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
          </lidar>
          <topic>{topic_prefix}/{idx}</topic>
        </sensor>
      </link>
    </model>
"""

def rt_lidar(idx, x, y, z=1.0, topic_prefix="/lidar/rt", steps=360):
    return f"""
    <model name="rt_lidar_{idx}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
        <sensor name="my_rt_lidar_{idx}" type="custom" gz:type="rt_lidar">
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <topic>{topic_prefix}/{idx}</topic>
          <gz:rt_lidar>
            <scan>
              <lasers>16</lasers>
              <steps>{steps}</steps>
              <vertical>
                <min_angle>-0.2617</min_angle>
                <max_angle>0.2617</max_angle>
              </vertical>
              <horizontal>
                <min_angle>-3.1415</min_angle>
                <max_angle>3.1415</max_angle>
              </horizontal>
            </scan>
            <range><min>0.1</min><max>100.0</max></range>
            <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
          </gz:rt_lidar>
        </sensor>
      </link>
    </model>
"""

def distribute_points(n, minx, maxx, miny, maxy):
    """Evenly distribute n points in a near-square grid across the bbox."""
    if n <= 0: return []
    kx = max(1, math.ceil(math.sqrt(n)))
    ky = max(1, math.ceil(n / kx))
    xs = [minx + (i + 0.5) * (maxx - minx) / kx for i in range(kx)]
    ys = [miny + (j + 0.5) * (maxy - miny) / ky for j in range(ky)]
    pts = []
    for j in range(ky):
        for i in range(kx):
            if len(pts) >= n: break
            pts.append((xs[i], ys[j]))
        if len(pts) >= n: break
    return pts

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--geometry", choices=["car", "box"], default="box")
    p.add_argument("--row", type=int, default=10)
    p.add_argument("--col", type=int, default=10)
    p.add_argument("--spacing", type=float, default=5.0)
    p.add_argument("--ground", type=float, default=100.0)
    p.add_argument("--sensor", choices=["gazebo", "rt"], default="rt")
    p.add_argument("--lidars", type=int, default=4)
    p.add_argument("--z_lidar", type=float, default=1.0)
    p.add_argument("--topic_prefix", type=str, default="/lidar")
    p.add_argument("--samples", type=int, default=360)
    args = p.parse_args()

    sys.stdout.write(world_header())
    sys.stdout.write(ground_plane(args.ground))

    # Build car grid centered at origin
    start_x = - (args.spacing * (args.col - 1) / 2.0)
    start_y = - (args.spacing * (args.row - 1) / 2.0)
    minx = start_x - args.spacing * 0.5
    maxx = start_x + args.spacing * (args.col - 0.5)
    miny = start_y - args.spacing * 0.5
    maxy = start_y + args.spacing * (args.row - 0.5)

    for j in range(args.row):
        for i in range(args.col):
            x = start_x + i * args.spacing
            y = start_y + j * args.spacing
            color = "blue" if ((i + j) % 2 == 0) else "red"
            if args.geometry == "box":
                sys.stdout.write(box(i, j, x, y, color))
            else:
                sys.stdout.write(hatch_model(i, j, x, y, color))

    # Lidar placement across grid footprint
    pts = distribute_points(args.lidars, minx, maxx, miny, maxy)
    for idx, (x, y) in enumerate(pts):
        if args.sensor == "gazebo":
            sys.stdout.write(gazebo_lidar(idx, x, y, args.z_lidar, topic_prefix=f"{args.topic_prefix}/gazebo", steps=args.samples))
        else:
            sys.stdout.write(rt_lidar(idx, x, y, args.z_lidar, topic_prefix=f"{args.topic_prefix}/rt", steps=args.samples))

    sys.stdout.write(world_footer())

if __name__ == "__main__":
    main()
