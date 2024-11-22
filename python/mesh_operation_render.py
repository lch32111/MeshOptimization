import trimesh
import numpy as np
from pathlib import Path

def get_triangle_edges(mesh:trimesh.Trimesh):
    """
    Get the edges of a triangle mesh
    """
    edges = mesh.edges_unique
    edge_vertices = mesh.vertices[edges]
    return edge_vertices
    
def show_wire_mesh(mesh:trimesh.Trimesh):
    """
    Show the wireframe of a mesh
    """
    edges = get_triangle_edges(mesh)
    scene = trimesh.Scene()
    scene.add_geometry(trimesh.load_path(edges))
    scene.add_geometry(trimesh.PointCloud(mesh.vertices, mesh.visual.vertex_colors))
    scene.show(line_settings={'point_size': 20})

def save_wire_mesh(source_path : Path, dest_path : Path):
    """
    Save the wireframe of a mesh
    """
    if dest_path.exists() == True:
        return

    mesh = trimesh.load_mesh(source_path, process=False)
    edges = get_triangle_edges(mesh)
    scene = trimesh.Scene()
    scene.add_geometry(trimesh.load_path(edges))
    scene.add_geometry(trimesh.PointCloud(mesh.vertices, mesh.visual.vertex_colors))

    kwargs = {
        'visible' : True,
        'background': [1.0, 1.0, 1.0, 1.0],
        'line_settings': {'point_size': 20},
    }
    png = scene.save_image(resolution=[640, 320], **kwargs)

    with open(dest_path, 'wb') as f:
        f.write(png)


path = Path('bin/Debug/mesh_test')

tests = path.glob('*.obj')

for test in tests:
    save_wire_mesh(test, test.with_suffix('.png'))