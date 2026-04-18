#!/usr/bin/env python3
"""
Generates a randomised Gazebo world file on every launch.

Writes:
  worlds/environment_foot.world  – loaded by Gazebo
  worlds/layout.json             – consumed by mission scripts at runtime

Placement rules
---------------
- Outer boundary walls (grid_border) stay fixed at ±10 m.
- Every other object is placed randomly with collision-avoidance rejection
  sampling so objects never overlap each other or the robot spawn points.
- Fireball 1 oscillates along world-X (amplitude ≈ 0.70 m); its spawn
  X-range is tightened so it never clips the walls or static obstacles.
- Fireball 2 oscillates along world-Y (amplitude ≈ 0.58 m); same treatment
  in Y.
- Gas zone and mannequin are treated as one unit: the mannequin is placed
  at a random position *inside* the gas cylinder.
"""

import json
import math
import os
import sys

import numpy as np

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
PKG_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_OUT = os.path.join(PKG_DIR, 'worlds', 'environment_foot.world')
LAYOUT_OUT = os.path.join(PKG_DIR, 'worlds', 'layout.json')
MEDIA_URI  = 'file:///home/ubuntu/catkin_ws/src/com760cw2_group6/media/materials'

# ---------------------------------------------------------------------------
# Arena geometry
# ---------------------------------------------------------------------------
WALL        = 10.0          # wall centre-line in world metres
WALL_THICK  = 0.2
INNER       = WALL - WALL_THICK  # 9.8 m – hard inner edge of walls

# Robot 0 spawns at (0, 0), Robot 1 at (3, 0)
ROBOTS          = [(0.0, 0.0), (3.0, 0.0)]
ROBOT_CLEARANCE = 2.5       # metres – minimum gap between any object and a robot

# ---------------------------------------------------------------------------
# Placement helpers
# ---------------------------------------------------------------------------

def _collides(x, y, radius, placed):
    for px, py, pr in placed:
        if math.hypot(x - px, y - py) < radius + pr:
            return True
    for rx, ry in ROBOTS:
        if math.hypot(x - rx, y - ry) < radius + ROBOT_CLEARANCE:
            return True
    return False


def try_place(x_lo, x_hi, y_lo, y_hi, radius, placed, tries=800):
    """
    Return a random (x, y) within the given bounds that does not collide
    with already-placed objects or robot spawns.  Returns None on failure.
    """
    for _ in range(tries):
        x = np.random.uniform(x_lo, x_hi)
        y = np.random.uniform(y_lo, y_hi)
        if not _collides(x, y, radius, placed):
            return float(x), float(y)
    return None


# ---------------------------------------------------------------------------
# Layout randomisation
# ---------------------------------------------------------------------------

def build_layout():
    placed = []  # list of (x, y, effective_radius)

    # ── Tunnel (6 m × 5 m corridor; bounding radius ≈ 3.9 m) ───────────────
    # Orientation is kept at 0 rad so the corridor always runs along X.
    # The tunnel pose is the centre of the structure; local walls are at ±2.5 m
    # in Y and the corridor extends ±3 m in X, so the tight bounding circle is
    # sqrt(3² + 2.5²) ≈ 3.9 m.
    TUNNEL_R   = 3.9
    TUNNEL_LIM = INNER - TUNNEL_R   # ≈ 5.9 m
    result = try_place(-TUNNEL_LIM, TUNNEL_LIM, -TUNNEL_LIM, TUNNEL_LIM,
                       TUNNEL_R, placed)
    tx, ty = result if result else (0.0, 5.5)
    placed.append((tx, ty, TUNNEL_R))

    # ── Oak tree (bounding radius ≈ 2.0 m) ─────────────────────────────────
    TREE_R   = 2.0
    TREE_LIM = INNER - TREE_R   # 7.8 m
    result = try_place(-TREE_LIM, TREE_LIM, -TREE_LIM, TREE_LIM,
                       TREE_R, placed)
    tree_x, tree_y = result if result else (-7.0, 7.0)
    placed.append((tree_x, tree_y, TREE_R))

    # ── 6 stones (bounding radius ≈ 1.2 m each) ────────────────────────────
    STONE_R   = 1.2
    STONE_LIM = INNER - STONE_R   # 8.6 m
    STONE_SPECS = [
        ((0.8, 1.2, 0.8), 0.4),
        ((1.0, 1.0, 1.0), 0.5),
        ((1.5, 0.7, 0.6), 0.3),
        ((0.6, 0.6, 0.8), 0.4),
        ((0.9, 0.9, 1.0), 0.5),
        ((2.0, 0.4, 0.6), 0.3),
    ]
    FALLBACK_STONES = [
        (-4.0,  3.0), (5.0,  4.0), (-7.0, -5.0),
        ( 2.0,  9.0), (-2.0, -8.5), (8.5, -2.0),
    ]
    stones = []
    for i, ((sx, sy, sz), z_off) in enumerate(STONE_SPECS):
        result = try_place(-STONE_LIM, STONE_LIM, -STONE_LIM, STONE_LIM,
                           STONE_R, placed)
        ox, oy = result if result else FALLBACK_STONES[i]
        rot = float(np.random.uniform(0.0, math.pi))
        stones.append({'x': ox, 'y': oy, 'z': z_off,
                       'sx': sx, 'sy': sy, 'sz': sz, 'rot': rot})
        placed.append((ox, oy, STONE_R))

    # ── Gas zone + mannequin (cylinder r = 2.5 m; bounding r = 3.2 m) ──────
    # The mannequin is placed at a random position *inside* the cylinder.
    GAS_R   = 3.2
    GAS_LIM = INNER - GAS_R   # 6.6 m
    result = try_place(-GAS_LIM, GAS_LIM, -GAS_LIM, GAS_LIM, GAS_R, placed)
    gas_x, gas_y = result if result else (2.0, -6.0)
    placed.append((gas_x, gas_y, GAS_R))

    victim_r   = float(np.random.uniform(0.2, 1.5))
    victim_ang = float(np.random.uniform(0.0, 2.0 * math.pi))
    victim_x   = gas_x + victim_r * math.cos(victim_ang)
    victim_y   = gas_y + victim_r * math.sin(victim_ang)
    victim_yaw = float(np.random.uniform(0.0, 2.0 * math.pi))

    # ── Fireball 1: oscillates along world-X, amplitude ≈ 0.70 m ───────────
    # Spawn X must satisfy: |x| + sphere_r(0.5) + amplitude(0.70) + margin(0.5) ≤ INNER
    # → |x| ≤ INNER − 1.70 = 8.10
    FB_R      = 2.5   # effective inter-object collision radius (accounts for oscillation sweep)
    FB1_X_LIM = INNER - 1.70   # 8.10 m
    FB1_Y_LIM = INNER - 1.0    # 8.80 m
    result = try_place(-FB1_X_LIM, FB1_X_LIM, -FB1_Y_LIM, FB1_Y_LIM,
                       FB_R, placed)
    fb1_x, fb1_y = result if result else (6.5, 5.0)
    placed.append((fb1_x, fb1_y, FB_R))

    # ── Fireball 2: oscillates along world-Y, amplitude ≈ 0.58 m ───────────
    # Spawn Y: |y| ≤ INNER − (0.5 + 0.58 + 0.5) = 8.22
    FB2_X_LIM = INNER - 1.0    # 8.80 m
    FB2_Y_LIM = INNER - 1.58   # 8.22 m
    result = try_place(-FB2_X_LIM, FB2_X_LIM, -FB2_Y_LIM, FB2_Y_LIM,
                       FB_R, placed)
    fb2_x, fb2_y = result if result else (-5.0, -5.0)
    placed.append((fb2_x, fb2_y, FB_R))

    return {
        'tunnel':    {'x': tx,       'y': ty},
        'tree':      {'x': tree_x,   'y': tree_y},
        'stones':    stones,
        'gas':       {'x': gas_x,    'y': gas_y},
        'victim':    {'x': victim_x, 'y': victim_y, 'yaw': victim_yaw},
        'fireball1': {'x': fb1_x,    'y': fb1_y},
        'fireball2': {'x': fb2_x,    'y': fb2_y},
    }


# ---------------------------------------------------------------------------
# SDF building blocks
# ---------------------------------------------------------------------------

def _stone_sdf(name, x, y, z, rot, sx, sy, sz):
    uri = MEDIA_URI
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z} 0 0 {rot:.4f}</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>
          <contact><ode><kp>1e+06</kp><kd>1.0</kd></ode></contact></surface>
        </collision>
        <visual name="v">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><script>
            <uri>{uri}/scripts</uri>
            <name>Apocalypse/StoneTexture</name>
          </script></material>
        </visual>
      </link>
    </model>"""


# ---------------------------------------------------------------------------
# Full world SDF writer
# ---------------------------------------------------------------------------

def write_world(layout):
    t   = layout['tunnel']
    tr  = layout['tree']
    g   = layout['gas']
    v   = layout['victim']
    f1  = layout['fireball1']
    f2  = layout['fireball2']
    uri = MEDIA_URI

    stones_xml = ''.join(
        _stone_sdf(f'random_stone_{i + 1}',
                   s['x'], s['y'], s['z'], s['rot'],
                   s['sx'], s['sy'], s['sz'])
        for i, s in enumerate(layout['stones'])
    )

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
<world name="hazard_world">
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>70</iters>
          <sor>1.3</sor>
        </solver>
      </ode>
    </physics>

    <scene>
      <ambient>0.4 0.2 0.1 1</ambient>
      <background>0.7 0.4 0.3 1</background>
      <shadows>true</shadows>
      <sky>
        <time>18.5</time>
        <sunrise>6.0</sunrise>
        <sunset>19.0</sunset>
        <clouds><speed>0.6</speed><humidity>0.5</humidity></clouds>
      </sky>
      <fog>
        <color>0.5 0.2 0.1 1</color>
        <type>linear</type>
        <start>15.0</start>
        <end>80.0</end>
      </fog>
    </scene>

    <light name="sun" type="directional">
      <pose>0 0 10 0.8 0 0</pose>
      <diffuse>0.9 0.5 0.2 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Tunnel spotlight follows the tunnel's randomised position -->
    <light name="tunnel_lamp" type="spot">
      <pose>{t['x']:.3f} {t['y']:.3f} 2.8 0 0 0</pose>
      <diffuse>0.6 0.5 0.2 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>6.0</range><constant>0.3</constant></attenuation>
      <direction>0 0 -1</direction>
      <spot><inner_angle>0.6</inner_angle><outer_angle>1.5</outer_angle></spot>
      <plugin name="light_control" filename="libgazebo_ros_light_control.so">
        <flicker>true</flicker>
      </plugin>
    </light>

    <!-- ═══════════ FIXED: ground and outer walls ═══════════ -->

    <model name="cracked_ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><script>
            <uri>{uri}/scripts</uri>
            <name>Apocalypse/GroundTexture</name>
          </script></material>
        </visual>
      </link>
    </model>

    <model name="grid_border">
      <static>true</static>
      <link name="link">
        <collision name="n_c"><pose>0 10 0.5 0 0 0</pose><geometry><box><size>20.5 0.2 1.0</size></box></geometry></collision>
        <visual    name="n_v"><pose>0 10 0.5 0 0 0</pose><geometry><box><size>20.5 0.2 1.0</size></box></geometry><material><script><uri>{uri}/scripts</uri><name>Apocalypse/StoneTexture</name></script></material></visual>
        <collision name="s_c"><pose>0 -10 0.5 0 0 0</pose><geometry><box><size>20.5 0.2 1.0</size></box></geometry></collision>
        <visual    name="s_v"><pose>0 -10 0.5 0 0 0</pose><geometry><box><size>20.5 0.2 1.0</size></box></geometry><material><script><uri>{uri}/scripts</uri><name>Apocalypse/StoneTexture</name></script></material></visual>
        <collision name="e_c"><pose>10 0 0.5 0 0 0</pose><geometry><box><size>0.2 20.5 1.0</size></box></geometry></collision>
        <visual    name="e_v"><pose>10 0 0.5 0 0 0</pose><geometry><box><size>0.2 20.5 1.0</size></box></geometry><material><script><uri>{uri}/scripts</uri><name>Apocalypse/StoneTexture</name></script></material></visual>
        <collision name="w_c"><pose>-10 0 0.5 0 0 0</pose><geometry><box><size>0.2 20.5 1.0</size></box></geometry></collision>
        <visual    name="w_v"><pose>-10 0 0.5 0 0 0</pose><geometry><box><size>0.2 20.5 1.0</size></box></geometry><material><script><uri>{uri}/scripts</uri><name>Apocalypse/StoneTexture</name></script></material></visual>
      </link>
    </model>

    <!-- ═══════════ RANDOMISED: interior objects ═══════════ -->

    <include>
      <name>corner_tree</name>
      <uri>model://oak_tree</uri>
      <pose>{tr['x']:.3f} {tr['y']:.3f} 0 0 0 0</pose>
      <static>true</static>
    </include>

    <model name="path_tunnel">
      <static>true</static>
      <!-- Tunnel orientation fixed at 0 rad; only XY position is randomised -->
      <pose>{t['x']:.3f} {t['y']:.3f} -0.2 0 0 0</pose>
      <link name="link">
        <collision name="wall_l"><pose>0 -2.5 1.6 0 0 0</pose><geometry><box><size>6.0 0.5 3.2</size></box></geometry></collision>
        <visual    name="v_l">  <pose>0 -2.5 1.6 0 0 0</pose><geometry><box><size>6.0 0.5 3.2</size></box></geometry><material><script><uri>{uri}/scripts</uri><name>Apocalypse/StoneTexture</name></script></material></visual>
        <collision name="wall_r"><pose>0  2.5 1.6 0 0 0</pose><geometry><box><size>6.0 0.5 3.2</size></box></geometry></collision>
        <visual    name="v_r">  <pose>0  2.5 1.6 0 0 0</pose><geometry><box><size>6.0 0.5 3.2</size></box></geometry><material><script><uri>{uri}/scripts</uri><name>Apocalypse/StoneTexture</name></script></material></visual>
        <collision name="roof">  <pose>0 0 3.2 0 0 0</pose><geometry><box><size>6.0 5.2 0.2</size></box></geometry></collision>
        <visual    name="v_roof"><pose>0 0 3.2 0 0 0</pose><geometry><box><size>6.0 5.2 0.2</size></box></geometry><material><script><uri>{uri}/scripts</uri><name>Apocalypse/StoneTexture</name></script></material></visual>
        <collision name="c_debris_1"><pose>-1.5 1.2 0.6 0.2 0.5 0.7</pose><geometry><box><size>1.2 0.6 0.8</size></box></geometry><surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface></collision>
        <visual    name="v_debris_1"><pose>-1.5 1.2 0.6 0.2 0.5 0.7</pose><geometry><box><size>1.2 0.6 0.8</size></box></geometry><material><script><uri>{uri}/scripts</uri><name>Apocalypse/StoneTexture</name></script></material></visual>
        <collision name="c_debris_2"><pose>1.5 -1.2 0.5 -0.3 0 0.4</pose><geometry><box><size>1.0 1.0 0.6</size></box></geometry><surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface></collision>
        <visual    name="v_debris_2"><pose>1.5 -1.2 0.5 -0.3 0 0.4</pose><geometry><box><size>1.0 1.0 0.6</size></box></geometry><material><script><uri>{uri}/scripts</uri><name>Apocalypse/StoneTexture</name></script></material></visual>
      </link>
    </model>

    <!-- Fireball 1 – oscillates along world-X (±0.70 m); spawn X clamped to ±8.10 m -->
    <model name="hazard_rolling_1">
      <pose>{f1['x']:.3f} {f1['y']:.3f} 0.5 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><sphere><radius>0.5</radius></sphere></geometry>
          <material>
            <script><uri>{uri}/scripts</uri><name>Apocalypse/FireballTexture</name></script>
            <emissive>1 0.3 0 1</emissive>
          </material>
        </visual>
        <collision name="c"><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>
      </link>
      <plugin name="object_controller" filename="libgazebo_ros_planar_move.so">
        <commandTopic>/hazard_1/cmd_vel</commandTopic>
        <odometryTopic>odom</odometryTopic>
        <odometryFrame>odom</odometryFrame>
        <odometryRate>20.0</odometryRate>
        <robotBaseFrame>link</robotBaseFrame>
      </plugin>
    </model>

    <!-- Fireball 2 – oscillates along world-Y (±0.58 m); spawn Y clamped to ±8.22 m -->
    <model name="hazard_rolling_2">
      <pose>{f2['x']:.3f} {f2['y']:.3f} 0.5 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><sphere><radius>0.5</radius></sphere></geometry>
          <material>
            <script><uri>{uri}/scripts</uri><name>Apocalypse/FireballTexture</name></script>
            <emissive>1 0.3 0 1</emissive>
          </material>
        </visual>
        <collision name="c"><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>
      </link>
      <plugin name="object_controller" filename="libgazebo_ros_planar_move.so">
        <commandTopic>/hazard_2/cmd_vel</commandTopic>
        <odometryTopic>odom</odometryTopic>
        <odometryFrame>odom</odometryFrame>
        <odometryRate>20.0</odometryRate>
        <robotBaseFrame>link</robotBaseFrame>
      </plugin>
    </model>

    <!-- Gas zone + mannequin treated as one unit; both positions from layout.json -->
    <model name="poison_gas_zone">
      <static>true</static>
      <pose>{g['x']:.3f} {g['y']:.3f} 0.6 0 0 0</pose>
      <link name="gas_link">
        <visual name="v">
          <geometry><cylinder><radius>2.5</radius><length>1.2</length></cylinder></geometry>
          <material>
            <ambient>0 1 0 0.4</ambient>
            <diffuse>0 1 0 0.4</diffuse>
            <emissive>0 0.2 0 1</emissive>
          </material>
        </visual>
        <collision name="gas_collision">
          <geometry><cylinder><radius>2.5</radius><length>1.2</length></cylinder></geometry>
          <surface><contact><collide_bitmask>0x00</collide_bitmask></contact></surface>
        </collision>
        <sensor name="gas_contact_sensor" type="contact">
          <always_on>true</always_on>
          <update_rate>20</update_rate>
          <contact><collision>gas_link::gas_collision</collision></contact>
          <plugin name="gas_plugin" filename="libgazebo_ros_bumper.so">
            <bumperTopicName>gas_touch</bumperTopicName>
            <frameName>world</frameName>
          </plugin>
        </sensor>
      </link>
    </model>

    <!-- Mannequin placed at a random position inside the gas cylinder -->
    <model name="gas_victim">
      <static>true</static>
      <pose>{v['x']:.3f} {v['y']:.3f} 0.05 1.57 0 {v['yaw']:.4f}</pose>
      <link name="link">
        <collision name="c">
          <pose>0 0.7 0.1 0 0 0</pose>
          <geometry><box><size>0.5 1.8 0.4</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><mesh><uri>model://person_standing/meshes/standing.dae</uri></mesh></geometry>
          <material><script>
            <uri>{uri}/scripts</uri>
            <name>Victim/Clothed</name>
          </script></material>
        </visual>
      </link>
    </model>

    <!-- ═══════════ RANDOMISED: stones ═══════════ -->
{stones_xml}

    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/hazard_world</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>
  </world>
</sdf>
"""

    with open(WORLD_OUT, 'w') as fh:
        fh.write(sdf)
    print(f'[generate_world] world  → {WORLD_OUT}')


def write_layout(layout):
    with open(LAYOUT_OUT, 'w') as fh:
        json.dump(layout, fh, indent=2)
    print(f'[generate_world] layout → {LAYOUT_OUT}')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    np.random.seed()      # OS entropy – no fixed seed
    layout = build_layout()
    write_world(layout)
    write_layout(layout)
    sys.exit(0)
