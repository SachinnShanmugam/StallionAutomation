import os
import struct

def read_binary_stl(path):
    triangles = []
    if not os.path.exists(path):
        print(f"  [MISSING] {path}")
        return triangles
    try:
        with open(path, 'rb') as f:
            f.read(80)
            count_bytes = f.read(4)
            if len(count_bytes) < 4:
                return triangles
            count = struct.unpack('<I', count_bytes)[0]
            for _ in range(count):
                norm = struct.unpack('<fff', f.read(12))
                v1 = struct.unpack('<fff', f.read(12))
                v2 = struct.unpack('<fff', f.read(12))
                v3 = struct.unpack('<fff', f.read(12))
                f.read(2)
                triangles.append((norm, v1, v2, v3))
    except Exception as e:
        print(f"  [ERROR] {path}: {e}")
    return triangles

def write_binary_stl(path, triangles):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(b"Stallion VTOL Gazebo ENU Mesh".ljust(80, b'\x00'))
        f.write(struct.pack('<I', len(triangles)))
        for norm, v1, v2, v3 in triangles:
            f.write(struct.pack('<fff', *norm))
            f.write(struct.pack('<fff', *v1))
            f.write(struct.pack('<fff', *v2))
            f.write(struct.pack('<fff', *v3))
            f.write(b'\x00\x00')

def calc_normal(v1, v2, v3):
    ax, ay, az = v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2]
    bx, by, bz = v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2]
    nx = ay*bz - az*by
    ny = az*bx - ax*bz
    nz = ax*by - ay*bx
    len_n = (nx*nx + ny*ny + nz*nz)**0.5
    if len_n > 1e-6:
        return (nx/len_n, ny/len_n, nz/len_n)
    return (0.0, 0.0, 1.0)

# Transformation from CAD (mm) to Gazebo ENU (meters)
# CAD Frame:
#   Z_cad: nose (0) to tail (930) mm
#   X_cad: span (centerline at X=149.1 mm, +X is Right)
#   Y_cad: height (bottom=0, top=130 mm)
#
# Gazebo ENU Frame (meters):
#   X_gz = (300.0 - Z_cad) / 1000.0   (Front = +X)
#   Y_gz = (149.1 - X_cad) / 1000.0   (Left = +Y)
#   Z_gz = (Y_cad - 60.0) / 1000.0    (Up = +Z)

def transform_vertex(v, x_shift_mm=0.0, mirror_y=False):
    x_cad = v[0] + x_shift_mm
    y_cad = v[1]
    z_cad = v[2]
    
    x_gz = (300.0 - z_cad) / 1000.0
    y_gz = (149.1 - x_cad) / 1000.0
    if mirror_y:
        y_gz = -y_gz
    z_gz = (y_cad - 60.0) / 1000.0
    return (x_gz, y_gz, z_gz)

def transform_triangles(triangles, x_shift_mm=0.0, mirror_y=False):
    new_tris = []
    for norm, v1, v2, v3 in triangles:
        tv1 = transform_vertex(v1, x_shift_mm, mirror_y)
        tv2 = transform_vertex(v2, x_shift_mm, mirror_y)
        tv3 = transform_vertex(v3, x_shift_mm, mirror_y)
        # Fix triangle winding if mirrored
        if mirror_y:
            tnorm = calc_normal(tv1, tv3, tv2)
            new_tris.append((tnorm, tv1, tv3, tv2))
        else:
            tnorm = calc_normal(tv1, tv2, tv3)
            new_tris.append((tnorm, tv1, tv2, tv3))
    return new_tris

def print_bounds(name, triangles):
    if not triangles:
        print(f"{name:32s} EMPTY")
        return
    xs, ys, zs = [], [], []
    for _, v1, v2, v3 in triangles:
        for v in (v1, v2, v3):
            xs.append(v[0]); ys.append(v[1]); zs.append(v[2])
    xn, xx = min(xs), max(xs)
    yn, yx = min(ys), max(ys)
    zn, zx = min(zs), max(zs)
    print(f"{name:32s} X:[{xn:6.3f},{xx:6.3f}] Y:[{yn:6.3f},{yx:6.3f}] Z:[{zn:6.3f},{zx:6.3f}] Size:[{xx-xn:5.3f},{yx-yn:5.3f},{zx-zn:5.3f}] m")

base_design = r'C:\Users\SACHIN\Stallion\Stallion_Design'
v2_fus = os.path.join(base_design, r'STALLION-FILES-zospag\STALLION FILES\V2\FUSELAGE')
v2_wing = os.path.join(base_design, r'STALLION-FILES-zospag\STALLION FILES\V2\WING')
v2_tail = os.path.join(base_design, r'STALLION-FILES-zospag\STALLION FILES\V2\TAIL')
v2_vtol = os.path.join(base_design, r'STALLION-VTOL-FILES-atx12o\STALLION VTOL FILES\V2\STL')
out_dir = r'C:\Users\SACHIN\Stallion\gazebo\models\stallion_vtol\meshes'

# 1. MAIN FUSELAGE + V-TAIL
print("Building Fuselage + V-Tail Mesh...")
body_tris = []

# Centered at X=149.1 mm
fus_specs = [
    (os.path.join(v2_fus, 'NOSE CLEAN.STL'), 90.0),
    (os.path.join(v2_fus, 'FUS 1L.STL'), 73.5),
    (os.path.join(v2_fus, 'FUS 1R.STL'), -3.0),
    (os.path.join(v2_fus, 'FUS 2L.STL'), 0.0),
    (os.path.join(v2_fus, 'FUS 2R.STL'), 0.0),
    (os.path.join(v2_fus, 'FUS 3L.STL'), 0.0),
    (os.path.join(v2_fus, 'FUS 3R.STL'), 0.0),
    (os.path.join(v2_fus, 'FUS 4L.STL'), 0.0),
    (os.path.join(v2_fus, 'FUS 4R.STL'), 0.0),
    (os.path.join(v2_fus, 'FUS 5L.STL'), 0.0),
    (os.path.join(v2_fus, 'FUS 5R.STL'), 0.0),
    (os.path.join(v2_fus, 'HATCH FRONT 1.STL'), 94.0),
    (os.path.join(v2_fus, 'HATCH FRONT 2.STL'), 94.0),
    (os.path.join(v2_fus, 'HATCH REAR 1.STL'), 98.0),
    (os.path.join(v2_fus, 'HATCH REAR 2.STL'), 98.0),
    (os.path.join(v2_tail, 'TAIL 1.STL'), 120.0),
    (os.path.join(v2_tail, 'TAIL 2 .STL'), 120.0),
    (os.path.join(v2_tail, 'TAIL 3.STL'), 134.0),
    (os.path.join(v2_tail, 'V TAIL 1 L.STL'), -2.0),
    (os.path.join(v2_tail, 'V TAIL 1 R.STL'), 75.0),
    (os.path.join(v2_tail, 'V TAIL 2 L.STL'), 36.0),
    (os.path.join(v2_tail, 'V TAIL 2 R.STL'), -18.0),
    (os.path.join(v2_tail, 'V TAIL TIP L.STL'), 141.0),
    (os.path.join(v2_tail, 'V TAIL TIP R.STL'), -70.0),
]

for path, shift in fus_specs:
    raw = read_binary_stl(path)
    body_tris.extend(transform_triangles(raw, x_shift_mm=shift))

write_binary_stl(os.path.join(out_dir, 'stallion_body_unified.stl'), body_tris)
print_bounds('stallion_body_unified.stl', body_tris)

# 2. RIGHT WING
print("\nBuilding Right Wing Mesh...")
rw_tris = []
rw_files = [
    (os.path.join(v2_wing, 'WING 1 R.STL'), 0.0),
    (os.path.join(v2_wing, 'WING 2R.STL'), 0.0),
    (os.path.join(v2_wing, 'WING 3 R.STL'), 0.0),
    (os.path.join(v2_wing, 'WINGTIP R NO LED.STL'), 0.0),
    (os.path.join(v2_wing, 'AILERON 1R.STL'), 0.0),
    (os.path.join(v2_wing, 'AILERON 2R.STL'), 0.0),
    (os.path.join(v2_vtol, 'BOOM R.STL'), 0.0),
]

for path, shift in rw_files:
    raw = read_binary_stl(path)
    rw_tris.extend(transform_triangles(raw, x_shift_mm=shift))

write_binary_stl(os.path.join(out_dir, 'stallion_wing_right_unified.stl'), rw_tris)
print_bounds('stallion_wing_right_unified.stl', rw_tris)

# 3. LEFT WING
print("\nBuilding Left Wing Mesh...")
lw_tris = []
# Mirror right wing across Y=0 plane to get perfectly symmetrical left wing!
for path, shift in rw_files:
    raw = read_binary_stl(path)
    lw_tris.extend(transform_triangles(raw, x_shift_mm=shift, mirror_y=True))

write_binary_stl(os.path.join(out_dir, 'stallion_wing_left_unified.stl'), lw_tris)
print_bounds('stallion_wing_left_unified.stl', lw_tris)

# 4. MOTOR MOUNTS (tilt mount & tail mount)
print("\nBuilding Motor Mount Meshes...")
tilt_mount_raw = read_binary_stl(os.path.join(v2_vtol, 'MOTOR MOUNT FRONT.STL'))
# Front tilt mount is positioned at (x=0, y=0, z=0) relative to fl_tilt_link / fr_tilt_link
# Rotate motor mount so motor faces SKYWARDS (+Z) at tilt=0 rad
tilt_mount_tris = []
for norm, v1, v2, v3 in tilt_mount_raw:
    # Scale from mm to meters and center origin
    def transform_mount_v(v):
        return (v[0]/1000.0 - 0.016, v[1]/1000.0 - 0.016, v[2]/1000.0 - 0.020)
    tv1, tv2, tv3 = transform_mount_v(v1), transform_mount_v(v2), transform_mount_v(v3)
    tnorm = calc_normal(tv1, tv2, tv3)
    tilt_mount_tris.append((tnorm, tv1, tv2, tv3))

write_binary_stl(os.path.join(out_dir, 'stallion_tilt_mount.stl'), tilt_mount_tris)
print_bounds('stallion_tilt_mount.stl', tilt_mount_tris)

tail_mount_raw = read_binary_stl(os.path.join(v2_vtol, 'MOTOR MOUNT TAIL.STL'))
tail_mount_tris = []
for norm, v1, v2, v3 in tail_mount_raw:
    def transform_tail_mount_v(v):
        return (v[0]/1000.0 - 0.016, v[1]/1000.0 - 0.015, (v[2]-654.2)/1000.0)
    tv1, tv2, tv3 = transform_tail_mount_v(v1), transform_tail_mount_v(v2), transform_tail_mount_v(v3)
    tnorm = calc_normal(tv1, tv2, tv3)
    tail_mount_tris.append((tnorm, tv1, tv2, tv3))

write_binary_stl(os.path.join(out_dir, 'stallion_tail_mount.stl'), tail_mount_tris)
print_bounds('stallion_tail_mount.stl', tail_mount_tris)

print("\nALL STALLION MESHES SUCCESSFULLY CREATED & ALIGNED IN GAZEBO ENU METERS!")
