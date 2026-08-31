import os
import struct

def merge_stls(input_files, output_file):
    total_triangles = 0
    all_triangles_data = bytearray()
    
    for fname in input_files:
        if not os.path.exists(fname):
            print(f"Warning: File {fname} not found!")
            continue
        with open(fname, 'rb') as f:
            header = f.read(80)
            if len(header) < 80:
                continue
            count_bytes = f.read(4)
            if len(count_bytes) < 4:
                continue
            num_triangles = struct.unpack('<I', count_bytes)[0]
            tri_data = f.read(num_triangles * 50)
            actual_triangles = len(tri_data) // 50
            total_triangles += actual_triangles
            all_triangles_data.extend(tri_data[:actual_triangles * 50])
            print(f"Added {actual_triangles} triangles from {os.path.basename(fname)}")
            
    header_80 = b"Stallion VTOL Merged Mesh".ljust(80, b'\x00')
    with open(output_file, 'wb') as f:
        f.write(header_80)
        f.write(struct.pack('<I', total_triangles))
        f.write(all_triangles_data)
    print(f"--> Saved {output_file} ({total_triangles} total triangles, {os.path.getsize(output_file)} bytes)\n")

mesh_dir = '/mnt/c/Users/SACHIN/Stallion/gazebo/models/stallion_vtol/meshes'

# 1. Fuselage STL list
fuselage_files = [
    os.path.join(mesh_dir, 'FUS 1.STL'),
    os.path.join(mesh_dir, 'FUS 2.STL'),
    os.path.join(mesh_dir, 'FUS 3.STL'),
    os.path.join(mesh_dir, 'FUS 4.STL'),
    os.path.join(mesh_dir, 'FUS 5.STL'),
    os.path.join(mesh_dir, 'NOSE.STL'),
    os.path.join(mesh_dir, 'HATCH FRONT 1.STL'),
    os.path.join(mesh_dir, 'HATCH REAR 1.STL'),
]
merge_stls(fuselage_files, os.path.join(mesh_dir, 'stallion_fuselage.stl'))

# 2. Left Wing STL list
left_wing_files = [
    os.path.join(mesh_dir, 'WING 1L VTOL.STL'),
    os.path.join(mesh_dir, 'WING 2L VTOL.STL'),
    os.path.join(mesh_dir, 'WING 3L VTOL.STL'),
    os.path.join(mesh_dir, 'AILERON 1L.STL'),
    os.path.join(mesh_dir, 'AILERON 2L.STL'),
]
merge_stls(left_wing_files, os.path.join(mesh_dir, 'stallion_wing_left.stl'))

# 3. Right Wing STL list
right_wing_files = [
    os.path.join(mesh_dir, 'WING 1R VTOL.STL'),
    os.path.join(mesh_dir, 'WING 2R VTOL.STL'),
    os.path.join(mesh_dir, 'WING 3R VTOL.STL'),
    os.path.join(mesh_dir, 'AILERON 1R.STL'),
    os.path.join(mesh_dir, 'AILERON 2R.STL'),
]
merge_stls(right_wing_files, os.path.join(mesh_dir, 'stallion_wing_right.stl'))

# 4. Tail STL list
tail_files = [
    os.path.join(mesh_dir, 'V TAIL 1 L.STL'),
    os.path.join(mesh_dir, 'V TAIL 1 R.STL'),
    os.path.join(mesh_dir, 'V TAIL 2 L.STL'),
    os.path.join(mesh_dir, 'V TAIL 2 R.STL'),
    os.path.join(mesh_dir, 'RUDDER 1L.STL'),
    os.path.join(mesh_dir, 'RUDDER 1R.STL'),
]
merge_stls(tail_files, os.path.join(mesh_dir, 'stallion_tail.stl'))
