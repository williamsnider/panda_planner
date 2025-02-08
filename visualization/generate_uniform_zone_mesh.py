import trimesh
import numpy as np
import json

# Define parameters (these should match MATLAB parameters)
params = {
    "sphere_radius": 0.7703,
    "sphere_origin": [0, 0, 0.3419],
    "sphere_cutoff_top": 0.9832,
    "sphere_cutoff_bottom": -0.2339,
    "cylinder_height": 0.25,
    "bottom_cylinder_radius": 0.6310,
    "top_cylinder_radius": 0.4537,
    "staging_box_width": 0.37,
    "staging_box_height": 1.2,
    "staging_box_length": 0.85,
    "staging_box_center": [-0.52, -0.05, 0.5],
}

# Save parameters for verification
param_file = "mesh_params.json"
with open(param_file, "w") as f:
    json.dump(params, f, indent=4)

# Create the sphere
sphere = trimesh.creation.icosphere(subdivisions=4, radius=params["sphere_radius"])
sphere.apply_translation(params["sphere_origin"])

# Define cutoff planes as thin boxes
cutoff_thickness = 0.01  # Small thickness to ensure a clean cut

# 1. Bottom cutoff plane at the bottom cylinder's most -z extent
bottom_plane = trimesh.creation.box([10, 10, cutoff_thickness])  # Large X-Y area, very thin Z
bottom_plane.apply_translation([0, 0, params["sphere_cutoff_bottom"] - cutoff_thickness / 2])

# 2. -X cutoff plane at the box’s most -x extent
x_plane = trimesh.creation.box([cutoff_thickness, 10, 10])  # Thin X, large Y-Z area
x_plane.apply_translation(
    [params["staging_box_center"][0] - params["staging_box_width"] / 2 - cutoff_thickness / 2, 0, 0]
)

# Boolean difference to cut the sphere
sphere_cut = trimesh.boolean.difference([sphere, bottom_plane, x_plane])
sphere_cut = sphere_cut.split()

# Keep the largest connected component of the sphere after cutting
if isinstance(sphere_cut, list):
    sphere_cut = max(sphere_cut, key=lambda m: m.volume)

# Create the bottom cylinder
bottom_cylinder = trimesh.primitives.Cylinder(radius=params["bottom_cylinder_radius"], height=params["cylinder_height"])
bottom_cylinder.apply_translation([0, 0, params["sphere_cutoff_bottom"] + params["cylinder_height"] / 2])

# Create the top cylinder
top_cylinder = trimesh.primitives.Cylinder(radius=params["top_cylinder_radius"], height=params["cylinder_height"])
top_cylinder.apply_translation([0, 0, params["sphere_cutoff_top"] + params["cylinder_height"] / 2])

# Create the box
box = trimesh.creation.box([params["staging_box_width"], params["staging_box_length"], params["staging_box_height"]])
box.apply_translation(params["staging_box_center"])

# Perform boolean union to combine all objects into a single solid mesh
merged_mesh = trimesh.boolean.union([sphere_cut, bottom_cylinder, top_cylinder, box])

# Export as STL
stl_filename = "uniform_zone_mesh.stl"
merged_mesh.export(stl_filename)

print(f"STL file saved as: {stl_filename}")
print(f"Parameter file saved as: {param_file}")

# Show the final mesh
merged_mesh.show()
