import trimesh
import numpy as np
from pathlib import Path
import struct

def read_randpts(path):
    point_path = Path(path)
    with open(point_path, 'rb') as f:
        point_count = int.from_bytes(f.read(4), byteorder='little', signed=True)
        f.read(1) # eat new line
        points = []
        for i in range(point_count):
            vx = struct.unpack('f', f.read(4))
            f.read(1) # eat space
            vy = struct.unpack('f', f.read(4))
            f.read(1) # eat space
            vz = struct.unpack('f', f.read(4))
            f.read(1) # eat space
            f.read(1) # eat new line

            points.append([vx, vy, vz])
    points = np.asarray(points, dtype=np.float32).squeeze()
    # trimesh.points.PointCloud(points).show()
    return points

# wobj = trimesh.load_mesh('cylindrical_cloth227.obj')
# cloth_key =  list(obj.geometry.keys())[0]
# cloth_obj = obj.geometry[cloth_key]
# cloth_obj.show()

path = Path(r'bin\Release\Opt2024_11_22_12_0_39')

randpts = read_randpts(path.joinpath('randpts.txt'))

after_dict = {
    'vertex_colors': np.array([0.2, 0.2, 0.2, 1.0], dtype=np.float32),
    'process': False
    }

kwargs = {
    'background': [1.0, 1.0, 1.0, 1.0],
    'resolution': (1000, 800),
    'flags': {'cull':False},
    'visible': True
}

save_kwargs = {
        'visible' : True,
        'background': [1.0, 1.0, 1.0, 1.0],
        'flags': {
            'cull':False,
            'wireframe':True,
            },
}

def save_mesh(mesh:trimesh.Trimesh, path:Path):
    

    png = mesh.scene().save_image(resolution=[1000, 800], **save_kwargs)

    with open(path, 'wb') as f:
        f.write(png)

def show_linked_edges(mesh:trimesh.Trimesh, vertex_index):
    links = np.where(mesh.edges_sparse.toarray()[vertex_index] == True)[0]

    vertex_colors = np.zeros((len(mesh.vertices), 4))
    vertex_colors[:, :] = np.array([0.2, 0.2, 0.2, 1.0], dtype=np.float32)
    vertex_colors[vertex_index] = np.array([1.0, 0.0, 0.0, 1.0], dtype=np.float32) 
    vertex_colors[links] = np.array([0.0, 0.0, 1.0, 1.0], dtype=np.float32)
    mesh = trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces, process=False, vertex_colors=vertex_colors)
    scene = trimesh.Scene()
    scene.add_geometry(mesh)
    scene.add_geometry(trimesh.points.PointCloud(randpts))
    scene.show(**kwargs)

def save_mesh_with_points(mesh:trimesh.Trimesh, points:np.ndarray, path:Path):
    scene = trimesh.Scene()
    scene.add_geometry(mesh)
    scene.add_geometry(trimesh.points.PointCloud(points, colors=np.array([1.0, 0.0, 0.0, 1.0], dtype=np.float32)))
    png = scene.save_image(resolution=[1000, 800], **save_kwargs)
    with open(path, 'wb') as f:
        f.write(png)

origin_obj = trimesh.load_mesh(path.joinpath('origin.obj'), **after_dict)
optimized_obj = trimesh.load_mesh(path.joinpath('optimized.obj'), **after_dict)

# target = trimesh.load_mesh(path.joinpath('simpliciesfit_after1.obj'), **after_dict)
# show_linked_edges(origin_obj, 0)
# show_linked_edges(target, 0)


localfit_obj_strs = [ str(p) for p in path.glob('localfit_after*.obj')]
localfit_objs = [trimesh.load_mesh(p, **after_dict) for p in localfit_obj_strs]

simplicesfit_obj_strs = [ str(p) for p in path.glob('simpliciesfit_after*.obj')]
simplicesfit_objs = [trimesh.load_mesh(p, **after_dict) for p in simplicesfit_obj_strs]

assert len(localfit_objs) == len(simplicesfit_objs)

iteration = len(localfit_objs)

print(f'Orignal Mesh : Vertices ({origin_obj.vertices.shape}), Faces ({origin_obj.faces.shape})')
print(f'Optimized Mesh : Vertices ({optimized_obj.vertices.shape}), Faces ({optimized_obj.faces.shape})')
for idx in range(iteration):
    lobj = localfit_objs[idx]
    sobj = simplicesfit_objs[idx]
    print(f'Local Fit Mesh {idx} : Vertices ({lobj.vertices.shape}), Faces ({lobj.faces.shape})')
    print(f'Simplicies Fit Mesh {idx} : Vertices ({sobj.vertices.shape}), Faces ({sobj.faces.shape})')

save_mesh_with_points(origin_obj, randpts, path.joinpath('origin_with_points.png'))
save_mesh(origin_obj, path.joinpath('origin.png'))
save_mesh(optimized_obj, path.joinpath('optimized.png'))

pos_diff = 5.0
max_width = pos_diff * iteration
min_pos = -max_width / 2.0

scene = trimesh.scene.Scene()
scene.add_geometry(origin_obj, transform=trimesh.transformations.translation_matrix((min_pos - pos_diff, 2.0, 0.0)))
scene.add_geometry(trimesh.points.PointCloud(randpts), transform=trimesh.transformations.translation_matrix((min_pos - pos_diff, 2.0, 0.0)))
scene.add_geometry(trimesh.points.PointCloud(randpts), transform=trimesh.transformations.translation_matrix((min_pos, 0.0, 0.0)))
for idx in range(iteration):
    lobj = localfit_objs[idx]
    sobj = simplicesfit_objs[idx]

    ltransform = trimesh.transformations.translation_matrix((min_pos + pos_diff * idx, 4.0, 0.0))
    stransform = trimesh.transformations.translation_matrix((min_pos + pos_diff * idx, 0.0, 0.0))

    scene.add_geometry(lobj, transform=ltransform)
    scene.add_geometry(sobj, transform=stransform)
scene.add_geometry(optimized_obj, transform=trimesh.transformations.translation_matrix((min_pos + pos_diff * iteration, 2.0, 0.0)))
scene.show(**kwargs)