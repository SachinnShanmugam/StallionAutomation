import struct, os, sys

def stl_bounds(path):
    try:
        with open(path, 'rb') as f:
            f.read(80)
            n = struct.unpack('<I', f.read(4))[0]
            xs, ys, zs = [], [], []
            for _ in range(n):
                f.read(12)
                for v in range(3):
                    x, y, z = struct.unpack('<fff', f.read(12))
                    xs.append(x); ys.append(y); zs.append(z)
                f.read(2)
        return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)
    except Exception as e:
        return None

base = '/mnt/c/Users/SACHIN/Stallion/gazebo/models/stallion_vtol/meshes'
files = [
    f'{base}/stallion_fuselage.stl',
    f'{base}/stallion_wing_left.stl',
    f'{base}/stallion_wing_right.stl',
    f'{base}/stallion_tail.stl',
    f'{base}/BOOM L.STL',
    f'{base}/BOOM R.STL',
    f'{base}/MOTOR MOUNT FRONT.STL',
    f'{base}/MOTOR MOUNT TAIL.STL',
    f'{base}/WING 1L VTOL.STL',
    f'{base}/WING 1R VTOL.STL',
    f'{base}/WING 2L VTOL.STL',
    f'{base}/WING 2R VTOL.STL',
    f'{base}/WING 3L VTOL.STL',
    f'{base}/WING 3R VTOL.STL',
]

print("Part                           Xmin    Xmax    Ymin    Ymax    Zmin    Zmax    Xsize   Ysize   Zsize   Xcenter Ycenter Zcenter")
print("-" * 140)
for f in files:
    b = stl_bounds(f)
    name = os.path.basename(f)
    if b:
        xn,xx,yn,yx,zn,zx = b
        print(f"{name[:30]:30s} {xn:7.1f} {xx:7.1f} {yn:7.1f} {yx:7.1f} {zn:7.1f} {zx:7.1f} {xx-xn:7.1f} {yx-yn:7.1f} {zx-zn:7.1f} {(xn+xx)/2:7.1f} {(yn+yx)/2:7.1f} {(zn+zx)/2:7.1f}")
    else:
        print(f"{name[:30]:30s} FAILED")
