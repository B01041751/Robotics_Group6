#!/usr/bin/env python3
"""Generates the Gazebo world file. Static layout every run except the gas zone
and mannequin, which are randomised into a clear spot each time."""
import random
import math
import os
import struct
import zlib

# ==================================================================== #
#  USER CONFIG — edit these two lines to choose entrances for each bot  #
#  Options: 'N'  'S'  'E'  'W'                                         #
# ==================================================================== #
BOT0_ENTRANCE   = 'W'
BOT1_ENTRANCE   = 'S'
SPAWN_CLEARANCE = 1.5   # metres inside the wall where robots spawn
OBSTACLE_INSET  = 3.5   # metres ahead of spawn — gives robot a clear run-up to the wall


def create_pavement_assets(pkg_path):
    """Write a pavement tile PNG and OGRE material script if not already present."""
    tex_path = os.path.join(pkg_path, 'media', 'materials', 'textures', 'pavement_tile.png')
    mat_path = os.path.join(pkg_path, 'media', 'materials', 'scripts', 'pavement.material')

    # --- PNG: 64x64 concrete tile with dark seam border ---
    size, border = 64, 3
    rows = []
    for y in range(size):
        row = []
        for x in range(size):
            if x < border or x >= size - border or y < border or y >= size - border:
                row.extend([110, 110, 110])  # dark grey seam
            else:
                row.extend([195, 195, 195])  # light grey tile
        rows.append(bytes([0]) + bytes(row))

    compressed = zlib.compress(b''.join(rows), 9)

    def png_chunk(tag, data):
        c = tag + data
        return struct.pack('>I', len(data)) + c + struct.pack('>I', zlib.crc32(c) & 0xffffffff)

    png = (b'\x89PNG\r\n\x1a\n'
           + png_chunk(b'IHDR', struct.pack('>IIBBBBB', size, size, 8, 2, 0, 0, 0))
           + png_chunk(b'IDAT', compressed)
           + png_chunk(b'IEND', b''))

    with open(tex_path, 'wb') as f:
        f.write(png)

    # --- OGRE material: tile every ~2m on the 100x100 outer ground ---
    material = """\
material Pavement/Tile
{
    technique
    {
        pass
        {
            texture_unit
            {
                texture pavement_tile.png
                scale 0.02 0.02
            }
        }
    }
}
"""
    with open(mat_path, 'w') as f:
        f.write(material)


def box_radius(w, d):
    return math.sqrt((w / 2) ** 2 + (d / 2) ** 2)


def _obstacle_pos(side, gap_coord):
    """Return (x, y) centre of the entry obstacle wall for this entrance side."""
    _off = 10.0 - SPAWN_CLEARANCE   # 8.5 — same inset as spawn
    if side in ('N', 'S'):
        sign = -1 if side == 'N' else 1
        return gap_coord, sign * (-_off + OBSTACLE_INSET)
    else:  # E or W
        sign = -1 if side == 'E' else 1
        return sign * (-_off + OBSTACLE_INSET), gap_coord


def entry_obstacle_wall(name, side, gap_coord, m):
    """Short wall OBSTACLE_INSET metres in front of a bot's spawn position."""
    ox, oy = _obstacle_pos(side, gap_coord)
    LENGTH, THICK, HEIGHT = 3.0, 0.2, 1.0
    sx, sy = (LENGTH, THICK) if side in ('N', 'S') else (THICK, LENGTH)
    surf = "<surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>"
    geo  = f"<geometry><box><size>{sx:.2f} {sy:.2f} {HEIGHT:.2f}</size></box></geometry>"
    return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{ox:.4f} {oy:.4f} {HEIGHT/2:.2f} 0 0 0</pose>
    <link name="link">
      <collision name="c">{geo}{surf}</collision>
      <visual name="v"><cast_shadows>false</cast_shadows>{geo}{m}</visual>
    </link>
  </model>"""


def dist2d(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def random_pos(placed, new_radius, margin=8.5, attempts=300):
    """Find a position clear of all placed objects (each entry is (x, y, radius))."""
    for _ in range(attempts):
        x = random.uniform(-margin + new_radius, margin - new_radius)
        y = random.uniform(-margin + new_radius, margin - new_radius)
        if all(dist2d((x, y), (p[0], p[1])) >= new_radius + p[2] for p in placed):
            return (x, y)
    raise RuntimeError(f"Could not place gas zone (radius={new_radius:.1f}) after {attempts} attempts")


def mat_script(uri, name):
    return f"<material><script><uri>{uri}</uri><name>{name}</name></script></material>"


ENTRANCE_W = 1.2   # gap width — comfortably fits the 0.5 m wide robots
WALL_H     = 1.0
WALL_THICK = 0.2
WALL_SPAN  = (-10.25, 10.25)  # full extent of each wall including corner overlap


def solid_wall(wall_id, fixed_coord, running_axis, m):
    """Return (collision_sdf, visual_sdf) for a full unbroken wall."""
    rmin, rmax = WALL_SPAN
    sc = (rmin + rmax) / 2   # centre = 0.0
    sl = rmax - rmin          # full length = 20.5
    if running_axis == 'x':
        cx, cy, cz = sc, fixed_coord, WALL_H / 2
        sx, sy, sz = sl, WALL_THICK, WALL_H
    else:
        cx, cy, cz = fixed_coord, sc, WALL_H / 2
        sx, sy, sz = WALL_THICK, sl, WALL_H
    geo  = f"<geometry><box><size>{sx:.4f} {sy} {sz}</size></box></geometry>"
    pose = f"<pose>{cx:.4f} {cy} {cz} 0 0 0</pose>"
    col = f'<collision name="{wall_id}_c">{pose}{geo}</collision>'
    vis = f'<visual name="{wall_id}_v">{pose}<cast_shadows>false</cast_shadows>{geo}{m}</visual>'
    return col, vis


def entrance_sign(name, side, gap_coord, label_color):
    """A post + coloured board on the inner face of the wall marking an entrance."""
    # Position: gap centre along the wall, offset 0.15 m inward from wall face
    inward = 0.15
    if side == 'N':
        sx, sy = (0.9, 0.08)
        px, py = gap_coord, 10.0 - inward
    elif side == 'S':
        sx, sy = (0.9, 0.08)
        px, py = gap_coord, -10.0 + inward
    elif side == 'E':
        sx, sy = (0.08, 0.9)
        px, py = 10.0 - inward, gap_coord
    else:  # W
        sx, sy = (0.08, 0.9)
        px, py = -10.0 + inward, gap_coord

    r, g, b = label_color
    # Posts flank the wide dimension of the board
    if side in ('N', 'S'):   # board wide along x
        pl_pose = f"{-sx/2 - 0.03:.3f} 0 0.65 0 0 0"
        pr_pose = f"{sx/2  + 0.03:.3f} 0 0.65 0 0 0"
    else:                    # board wide along y (E/W walls)
        pl_pose = f"0 {-sy/2 - 0.03:.3f} 0.65 0 0 0"
        pr_pose = f"0 {sy/2  + 0.03:.3f} 0.65 0 0 0"

    return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{px:.4f} {py:.4f} 0 0 0 0</pose>
    <link name="link">
      <visual name="post_l">
        <pose>{pl_pose}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><cylinder><radius>0.03</radius><length>1.3</length></cylinder></geometry>
        <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material>
      </visual>
      <visual name="post_r">
        <pose>{pr_pose}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><cylinder><radius>0.03</radius><length>1.3</length></cylinder></geometry>
        <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material>
      </visual>
      <visual name="board">
        <pose>0 0 1.45 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><box><size>{sx} {sy} 0.35</size></box></geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
          <emissive>{r*0.25:.2f} {g*0.25:.2f} {b*0.25:.2f} 1</emissive>
        </material>
      </visual>
    </link>
  </model>"""


def split_wall(wall_id, fixed_coord, running_axis, gap_centre, m):
    """Return (collisions_sdf, visuals_sdf) for a wall split by an entrance gap.

    running_axis: 'x' for N/S walls, 'y' for E/W walls.
    fixed_coord : y value for N/S walls, x value for E/W walls.
    """
    rmin, rmax = WALL_SPAN
    gap_l = gap_centre - ENTRANCE_W / 2
    gap_r = gap_centre + ENTRANCE_W / 2

    segs = []
    if gap_l > rmin + 0.05:
        sc = (rmin + gap_l) / 2
        sl = gap_l - rmin
        segs.append((sc, sl, "a"))
    if gap_r < rmax - 0.05:
        sc = (gap_r + rmax) / 2
        sl = rmax - gap_r
        segs.append((sc, sl, "b"))

    cols, viss = [], []
    for seg_c, seg_l, sfx in segs:
        name = f"{wall_id}_{sfx}"
        if running_axis == 'x':
            cx, cy, cz = seg_c, fixed_coord, WALL_H / 2
            sx, sy, sz = seg_l, WALL_THICK, WALL_H
        else:
            cx, cy, cz = fixed_coord, seg_c, WALL_H / 2
            sx, sy, sz = WALL_THICK, seg_l, WALL_H
        geo  = f"<geometry><box><size>{sx:.4f} {sy} {sz}</size></box></geometry>"
        pose = f"<pose>{cx:.4f} {cy} {cz} 0 0 0</pose>"
        cols.append(f'<collision name="{name}_c">{pose}{geo}</collision>')
        viss.append(f'<visual name="{name}_v">{pose}<cast_shadows>false</cast_shadows>{geo}{m}</visual>')

    return "\n      ".join(cols), "\n      ".join(viss)


def stone_block(idx, x, y, z, rx, ry, rz, sx, sy, sz, uri):
    m = mat_script(uri, "Apocalypse/StoneTexture")
    surf = "<surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction><contact><ode><kp>1e+06</kp><kd>1.0</kd></ode></contact></surface>"
    geo = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    return f"""
  <model name="random_stone_{idx}">
    <static>true</static>
    <pose>{x} {y} {z} {rx} {ry} {rz}</pose>
    <link name="link">
      <collision name="c">{geo}{surf}</collision>
      <visual name="v"><cast_shadows>false</cast_shadows>{geo}{m}</visual>
    </link>
  </model>"""


FIRE_POSITIONS = [
    (-2.0,  1.5),   # central-left  — early hazard on Bot 0's eastward path
    ( 3.5, -3.0),   # central-right — later hazard between center and gas zone
]
FIRE_RADIUS = 0.40   # fireball collision sphere radius


def fire_model(name, x, y, uri=""):
    """Static fireball sphere using the Apocalypse/FireballTexture OGRE material.

    This reuses the fireball.jpeg texture already in the project.  The original
    hazard_1/hazard_2 models used this texture with a PlanarMove plugin; here we
    drop the plugin so the ball sits in place.  A contact sensor on the sphere lets
    Group6_fireball_detector.py report robot hits, and a warm point light adds glow.
    """
    mat_block = ""
    if uri:
        mat_block = f"""
          <script>
            <uri>{uri}</uri>
            <name>Apocalypse/FireballTexture</name>
          </script>"""
    return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{x:.2f} {y:.2f} 0.55 0 0 0</pose>
    <link name="link">
      <!-- Sphere visual — fireball texture from the project's media folder -->
      <visual name="fireball_vis">
        <cast_shadows>false</cast_shadows>
        <geometry><sphere><radius>0.40</radius></sphere></geometry>
        <material>
          <ambient>1.0 0.5 0.0 1</ambient>
          <diffuse>1.0 0.4 0.0 1</diffuse>
          <emissive>0.9 0.3 0.0 1</emissive>{mat_block}
        </material>
      </visual>

      <!-- Collision sphere — solid obstacle for the robot -->
      <collision name="fireball_col">
        <geometry><sphere><radius>0.40</radius></sphere></geometry>
        <surface>
          <contact><collide_bitmask>0x01</collide_bitmask></contact>
        </surface>
      </collision>

      <!-- Contact sensor so Group6_fireball_detector.py can report hits -->
      <sensor name="{name}_contact" type="contact">
        <always_on>true</always_on>
        <update_rate>20</update_rate>
        <contact><collision>link::fireball_col</collision></contact>
        <plugin name="{name}_plugin" filename="libgazebo_ros_bumper.so">
          <bumperTopicName>/{name}/contact</bumperTopicName>
          <frameName>world</frameName>
        </plugin>
      </sensor>
    </link>
  </model>

  <!-- Warm orange glow light centred on the fireball -->
  <light name="{name}_glow" type="point">
    <pose>{x:.2f} {y:.2f} 0.55 0 0 0</pose>
    <diffuse>1.0 0.45 0.05 1</diffuse>
    <specular>0.4 0.15 0.0 1</specular>
    <attenuation>
      <range>6.0</range><constant>0.25</constant>
      <linear>0.12</linear><quadratic>0.04</quadratic>
    </attenuation>
    <cast_shadows>false</cast_shadows>
    <visualize>false</visualize>
  </light>"""


def generate_world(output_path):
    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    uri = f"file://{pkg_path}/media/materials/scripts"
    create_pavement_assets(pkg_path)

    # ------------------------------------------------------------------ #
    # Fixed layout — same every run
    # ------------------------------------------------------------------ #
    TUNNEL_POS  = (0.0,  7.8)   # right wall clips north boundary (wall_r at y=10.3)
    TREE_POS    = (8.0,  8.0)

    # (x, y, z, rx, ry, rz, sx, sy, sz)
    STONES = [
        (-4.0,  3.0, 0.4,  0.0,  0.0,  0.8,  0.8, 1.2, 0.8),
        ( 5.0,  4.0, 0.5,  0.2,  0.0,  1.2,  1.0, 1.0, 1.0),
        (-3.0, -7.0, 0.3,  0.0,  0.0, -0.5,  1.5, 0.7, 0.6),
        ( 2.0,  9.0, 0.4,  0.0,  0.0,  0.3,  0.6, 0.6, 0.8),
        (-2.0, -8.5, 0.5,  0.4,  0.4,  0.0,  0.9, 0.9, 1.0),
        ( 8.5, -2.0, 0.3,  0.0,  0.0,  1.57, 2.0, 0.4, 0.6),
    ]

    # ------------------------------------------------------------------ #
    # Only the two chosen entrances get a gap; all others stay solid
    # ------------------------------------------------------------------ #
    open_sides = {BOT0_ENTRANCE, BOT1_ENTRANCE}

    ent_coord = {}   # side -> gap centre along that wall's running axis
    if 'N' in open_sides:
        ent_coord['N'] = random.uniform(*random.choice([(-8.5, -4.5), (4.5, 7.0)]))
    if 'S' in open_sides:
        ent_coord['S'] = random.uniform(-8.0, 8.0)
    if 'E' in open_sides:
        ent_coord['E'] = random.uniform(-8.0, 8.0)
    if 'W' in open_sides:
        ent_coord['W'] = random.uniform(-8.0, 8.0)

    # ------------------------------------------------------------------ #
    # Build placed list from all fixed objects so gas zone avoids them
    # ------------------------------------------------------------------ #
    placed = [
        (0.0,  0.0,  0.5),   # centre clear zone (bots converge here after entry)
        (*TUNNEL_POS, box_radius(6.0, 5.2)),   # tunnel ~3.97m
        (*TREE_POS,   2.0),                    # oak tree
    ]
    for s in STONES:
        placed.append((s[0], s[1], box_radius(s[6], s[7])))

    # Entry obstacle walls — register centres so gas zone avoids them
    _ow = box_radius(3.0, 0.2)
    for _side in (BOT0_ENTRANCE, BOT1_ENTRANCE):
        _ox, _oy = _obstacle_pos(_side, ent_coord[_side])
        placed.append((_ox, _oy, _ow))

    # ------------------------------------------------------------------ #
    # Randomise only the gas zones + mannequins
    # ------------------------------------------------------------------ #
    N_GAS = 3
    GAS_R = 1.0   # exclusion radius per gas zone

    # Fire obstacles — gas zones must clear the full danger radius (1.5m) + gas radius
    FIRE_DANGER_RADIUS = 1.5
    for fx, fy in FIRE_POSITIONS:
        placed.append((fx, fy, FIRE_DANGER_RADIUS + GAS_R))

    gas_zones = []
    for _g in range(N_GAS):
        _gx, _gy = random_pos(placed, new_radius=GAS_R)
        gas_zones.append((_gx, _gy))
        placed.append((_gx, _gy, GAS_R * 2))   # mutual exclusion between zones

    # ------------------------------------------------------------------ #
    # Entrance wall centres + robot spawn positions (outside the wall)
    # ------------------------------------------------------------------ #
    _off = 10.0 - SPAWN_CLEARANCE   # inside the boundary
    def _ent(side):
        c = ent_coord[side]
        if side == 'N': return dict(wall_x=c,    wall_y= 10.0, spawn_x=c,    spawn_y= _off, yaw=-1.5708)
        if side == 'S': return dict(wall_x=c,    wall_y=-10.0, spawn_x=c,    spawn_y=-_off, yaw= 1.5708)
        if side == 'E': return dict(wall_x= 10.0, wall_y=c,   spawn_x= _off, spawn_y=c,    yaw= 3.1416)
        if side == 'W': return dict(wall_x=-10.0, wall_y=c,   spawn_x=-_off, spawn_y=c,    yaw= 0.0)
    ent0 = _ent(BOT0_ENTRANCE)
    ent1 = _ent(BOT1_ENTRANCE)

    # Reserve spawn positions so random_pos() never places objects there
    placed.append((ent0['spawn_x'], ent0['spawn_y'], SPAWN_CLEARANCE))
    placed.append((ent1['spawn_x'], ent1['spawn_y'], SPAWN_CLEARANCE))

    # Write shell-sourceable env so run.bash can pass values to roslaunch
    env_path = os.path.join(os.path.dirname(output_path), 'spawn_config.env')
    with open(env_path, 'w') as f:
        f.write(
            f"export BOT0_X={ent0['spawn_x']:.4f}\n"
            f"export BOT0_Y={ent0['spawn_y']:.4f}\n"
            f"export BOT0_Z=0.1500\n"
            f"export BOT0_YAW={ent0['yaw']:.4f}\n"
            f"export BOT0_WALL_SIDE={BOT0_ENTRANCE}\n"
            f"export BOT0_WALL_X={ent0['wall_x']:.4f}\n"
            f"export BOT0_WALL_Y={ent0['wall_y']:.4f}\n"
            f"export BOT1_X={ent1['spawn_x']:.4f}\n"
            f"export BOT1_Y={ent1['spawn_y']:.4f}\n"
            f"export BOT1_Z=0.1500\n"
            f"export BOT1_YAW={ent1['yaw']:.4f}\n"
            f"export BOT1_WALL_SIDE={BOT1_ENTRANCE}\n"
            f"export BOT1_WALL_X={ent1['wall_x']:.4f}\n"
            f"export BOT1_WALL_Y={ent1['wall_y']:.4f}\n"
            + "".join(
                f"export GAS{i+1}_X={gx:.4f}\nexport GAS{i+1}_Y={gy:.4f}\n"
                for i, (gx, gy) in enumerate(gas_zones)
            )
            + "".join(
                f"export FIRE{i+1}_X={fx:.4f}\nexport FIRE{i+1}_Y={fy:.4f}\n"
                for i, (fx, fy) in enumerate(FIRE_POSITIONS)
            )
        )

    # ------------------------------------------------------------------ #
    # Build SDF
    # ------------------------------------------------------------------ #
    stones_sdf = "".join(stone_block(i + 1, *s, uri) for i, s in enumerate(STONES))

    m_stone  = mat_script(uri, "Apocalypse/StoneTexture")
    m_ground = mat_script(uri, "Apocalypse/GroundTexture")

    # Entry obstacle walls (one per bot, 3.5 m in front of each spawn)
    obs0_sdf = entry_obstacle_wall('entry_obstacle_bot0', BOT0_ENTRANCE, ent_coord[BOT0_ENTRANCE], m_stone)
    obs1_sdf = entry_obstacle_wall('entry_obstacle_bot1', BOT1_ENTRANCE, ent_coord[BOT1_ENTRANCE], m_stone)

    # All boundary walls solid — robots spawn inside, signs mark entrance positions
    n_c, n_v = solid_wall('n',  10.0, 'x', m_stone)
    s_c, s_v = solid_wall('s', -10.0, 'x', m_stone)
    e_c, e_v = solid_wall('e',  10.0, 'y', m_stone)
    w_c, w_v = solid_wall('w', -10.0, 'y', m_stone)

    # Entrance sign boards — blue for Bot 0, orange for Bot 1
    sign0_sdf = entrance_sign('entrance_sign_bot0', BOT0_ENTRANCE,
                              ent_coord[BOT0_ENTRANCE], (0.0, 0.45, 1.0))
    sign1_sdf = entrance_sign('entrance_sign_bot1', BOT1_ENTRANCE,
                              ent_coord[BOT1_ENTRANCE], (1.0, 0.55, 0.0))

    # Static fire obstacles
    fires_sdf = "".join(
        fire_model(f"fire_{i+1}", fx, fy, uri=uri)
        for i, (fx, fy) in enumerate(FIRE_POSITIONS)
    )

    # Gas zones SDF — one cylinder + mannequin per zone
    _gas_blocks = []
    for i, (gx, gy) in enumerate(gas_zones):
        _gas_blocks.append(f"""
  <model name="gas_zone_{i+1}">
    <static>true</static>
    <pose>{gx:.2f} {gy:.2f} 0.05 0 0 0</pose>
    <link name="link">
      <visual name="v">
        <cast_shadows>false</cast_shadows>
        <geometry><cylinder><radius>1.0</radius><length>0.1</length></cylinder></geometry>
        <material><ambient>0 0.8 0 0.5</ambient><diffuse>0 0.9 0 0.5</diffuse><emissive>0 0.4 0 1</emissive></material>
      </visual>
      <collision name="c">
        <geometry><cylinder><radius>1.0</radius><length>0.1</length></cylinder></geometry>
        <surface><contact><collide_bitmask>0x0000</collide_bitmask></contact><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface>
      </collision>
      <sensor name="contact_{i+1}" type="contact">
        <always_on>true</always_on>
        <update_rate>20</update_rate>
        <contact><collision>link::c</collision></contact>
        <plugin name="bumper_{i+1}" filename="libgazebo_ros_bumper.so">
          <bumperTopicName>/gas_{i+1}/contact</bumperTopicName>
          <frameName>world</frameName>
        </plugin>
      </sensor>
    </link>
  </model>

  <model name="gas_victim_{i+1}">
    <static>true</static>
    <pose>{gx:.2f} {gy:.2f} 0.05 1.57 0 0</pose>
    <link name="link">
      <visual name="v">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <mesh>
            <uri>model://person_standing/meshes/standing.dae</uri>
            <scale>0.5 0.5 0.5</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>""")
    gas_zones_sdf = "".join(_gas_blocks)

    world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
<world name="hazard_world">

  <physics type="ode">
    <real_time_update_rate>1000.0</real_time_update_rate>
    <max_step_size>0.001</max_step_size>
    <real_time_factor>0</real_time_factor>
    <ode><solver><type>quick</type><iters>70</iters><sor>1.3</sor></solver></ode>
  </physics>

  <scene>
    <ambient>0.4 0.2 0.1 1</ambient>
    <background>0.7 0.4 0.3 1</background>
    <shadows>false</shadows>
    <sky>
      <time>18.5</time><sunrise>6.0</sunrise><sunset>19.0</sunset>
      <clouds><speed>0.6</speed><humidity>0.5</humidity></clouds>
    </sky>
  </scene>

  <light name="sun" type="directional">
    <pose>0 0 10 0.8 0 0</pose>
    <diffuse>0.9 0.5 0.2 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>0 0 -1</direction>
    <cast_shadows>false</cast_shadows>
  </light>

  <light name="tunnel_lamp" type="point">
    <pose>0 7.8 2.95 0 0 0</pose>
    <diffuse>1.0 0.95 0.6 1</diffuse>
    <specular>0.4 0.4 0.4 1</specular>
    <attenuation><range>8.0</range><constant>0.3</constant><linear>0.1</linear><quadratic>0.05</quadratic></attenuation>
    <cast_shadows>false</cast_shadows>
    <visualize>false</visualize>
  </light>

  <!-- Outer pavement ground — full 100x100, tiled concrete look -->
  <model name="outer_ground">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        {mat_script(uri, "Pavement/Tile")}
      </visual>
    </link>
  </model>

  <!-- Inner textured ground — 20x20 inside boundary, sits just above outer ground -->
  <model name="cracked_ground">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <pose>0 0 0.001 0 0 0</pose>
        <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        {m_ground}
      </visual>
    </link>
  </model>

  <model name="grid_border">
    <static>true</static>
    <link name="link">
      {n_c}
      {n_v}
      {s_c}
      {s_v}
      {e_c}
      {e_v}
      {w_c}
      {w_v}
    </link>
  </model>

  <include>
    <name>corner_tree</name>
    <uri>model://oak_tree</uri>
    <pose>8.0 8.0 -0.8 0 0 0</pose>
    <static>true</static>
  </include>

  <model name="path_tunnel">
    <static>true</static>
    <pose>0.0 7.8 -0.2 0 0 0</pose>
    <link name="link">
      <collision name="wall_l"><pose>0 -2.5 1.6 0 0 0</pose><geometry><box><size>6.0 0.5 3.2</size></box></geometry></collision>
      <visual name="v_l"><cast_shadows>false</cast_shadows><pose>0 -2.5 1.6 0 0 0</pose><geometry><box><size>6.0 0.5 3.2</size></box></geometry>{m_stone}</visual>
      <collision name="wall_r"><pose>0 2.5 1.6 0 0 0</pose><geometry><box><size>6.0 0.5 3.2</size></box></geometry></collision>
      <visual name="v_r"><cast_shadows>false</cast_shadows><pose>0 2.5 1.6 0 0 0</pose><geometry><box><size>6.0 0.5 3.2</size></box></geometry>{m_stone}</visual>
      <collision name="roof"><pose>0 0 3.2 0 0 0</pose><geometry><box><size>6.0 5.2 0.2</size></box></geometry></collision>
      <visual name="v_roof"><cast_shadows>false</cast_shadows><pose>0 0 3.2 0 0 0</pose><geometry><box><size>6.0 5.2 0.2</size></box></geometry>{m_stone}</visual>
      <collision name="c_debris_1"><pose>-1.5 1.2 0.6 0.2 0.5 0.7</pose><geometry><box><size>1.2 0.6 0.8</size></box></geometry><surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface></collision>
      <visual name="v_debris_1"><cast_shadows>false</cast_shadows><pose>-1.5 1.2 0.6 0.2 0.5 0.7</pose><geometry><box><size>1.2 0.6 0.8</size></box></geometry>{m_stone}</visual>
      <collision name="c_debris_2"><pose>1.5 -1.2 0.5 -0.3 0 0.4</pose><geometry><box><size>1.0 1.0 0.6</size></box></geometry><surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface></collision>
      <visual name="v_debris_2"><cast_shadows>false</cast_shadows><pose>1.5 -1.2 0.5 -0.3 0 0.4</pose><geometry><box><size>1.0 1.0 0.6</size></box></geometry>{m_stone}</visual>
    </link>
  </model>

  {stones_sdf}

  {sign0_sdf}
  {sign1_sdf}

  <!-- Static fire obstacles -->
  {fires_sdf}

  <!-- Entry obstacle walls — one in front of each bot spawn -->
  {obs0_sdf}
  {obs1_sdf}

  <!-- Gas zones — 3 independently randomised positions (radius 1.0m each) -->
  {gas_zones_sdf}

  <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros><namespace>/hazard_world</namespace></ros>
    <update_rate>10.0</update_rate>
  </plugin>

  <gui fullscreen="false">
    <camera name="user_camera">
      <pose>16.406 -17.062 19.287 0.0 0.7636 2.2402</pose>
    </camera>
  </gui>

</world>
</sdf>
"""

    with open(output_path, 'w') as f:
        f.write(world)

    print(f"[World Generator] World written to: {output_path}")
    for i, (gx, gy) in enumerate(gas_zones):
        print(f"  Gas zone {i+1}: ({gx:.1f}, {gy:.1f})")
    for side, coord in ent_coord.items():
        print(f"  Open entrance {side}: gap centre = {coord:.2f}")
    print(f"  Bot0 ({BOT0_ENTRANCE} entrance): spawn ({ent0['spawn_x']:.2f}, {ent0['spawn_y']:.2f})")
    _o0x, _o0y = _obstacle_pos(BOT0_ENTRANCE, ent_coord[BOT0_ENTRANCE])
    print(f"    obstacle wall at ({_o0x:.2f}, {_o0y:.2f}) — {OBSTACLE_INSET:.1f}m ahead of spawn")
    print(f"  Bot1 ({BOT1_ENTRANCE} entrance): spawn ({ent1['spawn_x']:.2f}, {ent1['spawn_y']:.2f})")
    _o1x, _o1y = _obstacle_pos(BOT1_ENTRANCE, ent_coord[BOT1_ENTRANCE])
    print(f"    obstacle wall at ({_o1x:.2f}, {_o1y:.2f}) — {OBSTACLE_INSET:.1f}m ahead of spawn")
    print(f"  Spawn config written to: {env_path}")


if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    output = os.path.join(pkg_dir, 'worlds', 'environment_foot.world')
    generate_world(output)
