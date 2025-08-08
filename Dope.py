#!/usr/bin/env python3

import blenderproc as bp  # must be first!
from blenderproc.python.utility.Utility import Utility
import bpy
import glob
import json
import numpy as np
import os
from PIL import Image, ImageDraw
from pyquaternion import Quaternion
import random
import sys
from tqdm import tqdm

# ANSI colors
GREEN = "\033[1;32m"
BLUE = "\033[1;34m"
RED = "\033[1;31m"
RESET = "\033[0m"

# -----------------------------------------------------------
# Load OBJ models only (with MTL if available)
# -----------------------------------------------------------
def load_objects_from_folder(folder_path):
    objects = []
    objects_data = []

    obj_files = [f for f in os.listdir(folder_path) if f.lower().endswith('.obj')]
    obj_files.sort()

    print(f"{GREEN}Found {len(obj_files)} OBJ models in {folder_path}{RESET}")

    for idx, obj_file in enumerate(obj_files):
        model_path = os.path.join(folder_path, obj_file)
        print(f"{BLUE}Loading model: {model_path}{RESET}")

        obj = bp.loader.load_obj(model_path)[0]
        obj.set_cp("category_id", idx + 1)

        # Assign random material if none
        if not obj.blender_obj.data.materials:
            mat = bpy.data.materials.new(name="RandomMat")
            mat.use_nodes = True
            bsdf = mat.node_tree.nodes.get("Principled BSDF")
            if bsdf:
                bsdf.inputs["Base Color"].default_value = (
                    random.random(), random.random(), random.random(), 1.0
                )
            obj.blender_obj.data.materials.clear()
            obj.blender_obj.data.materials.append(mat)

        objects.append(obj)
        objects_data.append({
            'class': os.path.splitext(obj_file)[0],
            'name': os.path.splitext(obj_file)[0] + "_" + str(idx).zfill(3),
            'id': idx + 1
        })

    return objects, objects_data

# -----------------------------------------------------------
# DOPE helper functions
# -----------------------------------------------------------
def get_cuboid_image_space(mesh, camera):
    import cv2
    bbox = mesh.get_bound_box()
    centroid = np.mean(bbox, axis=0)
    cam_pose = np.linalg.inv(camera.get_camera_pose())
    tvec = -cam_pose[0:3, 3]
    rvec = -cv2.Rodrigues(cam_pose[0:3, 0:3])[0]
    K = camera.get_intrinsics_as_K_matrix()

    dope_order = [6, 2, 1, 5, 7, 3, 0, 4]
    cuboid = [None for _ in range(9)]
    for ii in range(8):
        cuboid[dope_order[ii]] = cv2.projectPoints(
            bbox[ii], rvec, tvec, K, np.array([])
        )[0][0][0]
    cuboid[8] = cv2.projectPoints(centroid, rvec, tvec, K, np.array([]))[0][0][0]

    return np.array(cuboid, dtype=float).tolist()

def write_json(outf, width, height, min_pixels, camera, objects, objects_data, seg_map):
    cam_xform = camera.get_camera_pose()
    eye = -cam_xform[0:3, 3]
    at = -cam_xform[0:3, 2]
    up = cam_xform[0:3, 0]
    K = camera.get_intrinsics_as_K_matrix()

    data = {
        "camera_data": {
            "width": width,
            "height": height,
            "camera_look_at": {"eye": eye.tolist(), "at": at.tolist(), "up": up.tolist()},
            "intrinsics": {"fx": K[0][0], "fy": K[1][1], "cx": K[2][0], "cy": K[2][1]}
        },
        "objects": []
    }

    for ii, oo in enumerate(objects):
        idx = ii + 1
        num_pixels = int(np.sum((seg_map == idx)))
        if num_pixels < min_pixels:
            continue
        projected_keypoints = get_cuboid_image_space(oo, camera)
        data['objects'].append({
            'class': objects_data[ii]['class'],
            'name': objects_data[ii]['name'],
            'visibility': num_pixels,
            'projected_cuboid': projected_keypoints,
            'location': objects_data[ii]['location'],
            'quaternion_xyzw': objects_data[ii]['quaternion_xyzw']
        })

    with open(outf, "w") as write_file:
        json.dump(data, write_file, indent=4)

# -----------------------------------------------------------
# Background helpers
# -----------------------------------------------------------
def load_background_images(backgrounds_folder):
    image_types = ('*.jpg', '*.jpeg', '*.png', '*.hdr', '*.HDR')
    backdrop_images = []
    if not os.path.exists(backgrounds_folder):
        print(f"{RED}Background folder '{backgrounds_folder}' not found{RESET}")
        return backdrop_images
    for ext in image_types:
        backdrop_images.extend(glob.glob(os.path.join(backgrounds_folder, '**', ext), recursive=True))
    return backdrop_images

def detect_is_hdr(backdrop_images):
    if not backdrop_images:
        return False, None
    background_path = backdrop_images[random.randint(0, len(backdrop_images) - 1)]
    is_hdr = os.path.splitext(background_path)[1].lower() == ".hdr"
    return is_hdr, background_path

def setup_hdr_background(background_path, is_hdr=False):
    if is_hdr:
        strength = random.random() + 0.5
        rotation = [random.random()*0.2 - 0.1 for _ in range(3)]
        set_world_background_hdr(background_path, strength, rotation)
        bp.renderer.set_output_format(enable_transparency=False)
    else:
        bp.renderer.set_output_format(enable_transparency=True)

def set_world_background_hdr(filename, strength=1.0, rotation_euler=None):
    if rotation_euler is None:
        rotation_euler = [0.0, 0.0, 0.0]
    nodes = bpy.context.scene.world.node_tree.nodes
    links = bpy.context.scene.world.node_tree.links
    texture_node = nodes.new(type="ShaderNodeTexEnvironment")
    texture_node.image = bpy.data.images.load(filename, check_existing=True)
    background_node = Utility.get_the_one_node_with_type(nodes, "Background")
    links.new(texture_node.outputs["Color"], background_node.inputs["Color"])
    background_node.inputs["Strength"].default_value = strength
    mapping_node = nodes.new("ShaderNodeMapping")
    tex_coords_node = nodes.new("ShaderNodeTexCoord")
    links.new(tex_coords_node.outputs["Generated"], mapping_node.inputs["Vector"])
    links.new(mapping_node.outputs["Vector"], texture_node.inputs["Vector"])
    mapping_node.inputs["Rotation"].default_value = rotation_euler

# -----------------------------------------------------------
# Main
# -----------------------------------------------------------
def main():
    models_folder = "models/"
    backgrounds_folder = "backgrounds/"
    output_dir = "datasets/"
    nb_frames = 5
    width, height = 512, 512
    scale = 0.01
    min_pixels = 1

    os.makedirs(output_dir, exist_ok=True)
    dope_data_folder = os.path.join(output_dir, "dope_data")
    os.makedirs(dope_data_folder, exist_ok=True)

    bp.init()
    bp.renderer.set_output_format('PNG')
    bp.renderer.set_render_devices(desired_gpu_ids=[0])

    objects, objects_data = load_objects_from_folder(models_folder)
    if not objects:
        print(f"{RED}No OBJ models found. Exiting.{RESET}")
        return

    backdrop_images = load_background_images(backgrounds_folder)

    cam_pose = bp.math.build_transformation_mat([0, -6, 1.5], [np.pi / 2, 0, 0])
    bp.camera.add_camera_pose(cam_pose)
    bp.camera.set_resolution(width, height)
    bp.camera.set_intrinsics_from_blender_params(lens=0.785398, lens_unit='FOV', clip_start=1.0, clip_end=1000.0)

    bp.renderer.enable_depth_output(activate_antialiasing=False)
    bp.renderer.set_max_amount_of_samples(50)

    for frame in tqdm(range(nb_frames), desc="Rendering DOPE frames"):
        for idx, oo in enumerate(objects):
            pose = np.eye(4)
            pose[0:3, 3] = np.array([0.0, 5.0, 0.0])
            pose[0:3, 0:3] = np.eye(3)
            oo.set_local2world_mat(pose)
            oo.set_scale([scale] * 3)

            cam_rel_pose = np.linalg.inv(bp.camera.get_camera_pose()) @ pose
            objects_data[idx]['location'] = cam_rel_pose[0:3, 3].tolist()
            q = Quaternion(matrix=cam_rel_pose[0:3, 0:3])
            objects_data[idx]['quaternion_xyzw'] = [q.x, q.y, q.z, q.w]

        is_hdr, background_path = detect_is_hdr(backdrop_images)
        setup_hdr_background(background_path, is_hdr=is_hdr)

        segs = bp.renderer.render_segmap()
        data = bp.renderer.render()
        im = Image.fromarray(data['colors'][0])
        im.save(os.path.join(dope_data_folder, f"{frame:06}.png"))

        write_json(os.path.join(dope_data_folder, f"{frame:06}.json"),
                   width, height, min_pixels, bp.camera, objects, objects_data, segs['class_segmaps'][0])

    print(f"{GREEN}DOPE data saved in: {dope_data_folder}{RESET}")

if __name__ == "__main__":
    main()
