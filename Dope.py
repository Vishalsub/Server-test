#!/usr/bin/env python3
import blenderproc as bp
import bpy
import os
import glob
import random
import numpy as np
from tqdm import tqdm
from PIL import Image
from pyquaternion import Quaternion
import json

GREEN = "\033[1;32m"
RED = "\033[1;31m"
RESET = "\033[0m"

# ---------------- OBJ Loader ---------------- #
def load_obj_models(folder_path):
    objects = []
    objects_data = []

    obj_files = [f for f in os.listdir(folder_path) if f.lower().endswith('.obj')]
    obj_files.sort()

    print(f"{GREEN}Found {len(obj_files)} OBJ models in {folder_path}{RESET}")
    if not obj_files:
        return [], []

    for idx, obj_file in enumerate(obj_files):
        model_path = os.path.join(folder_path, obj_file)
        obj_list = bp.loader.load_obj(model_path)
        if not obj_list:
            print(f"{RED}Failed to load: {obj_file}{RESET}")
            continue

        obj = obj_list[0]
        obj.set_cp("category_id", idx + 1)

        # Apply random color if no texture
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
            "class": os.path.splitext(obj_file)[0],
            "name": os.path.splitext(obj_file)[0] + "_" + str(idx).zfill(3),
            "id": idx + 1
        })

    return objects, objects_data

# ---------------- DOPE JSON Writer ---------------- #
def get_cuboid_image_space(mesh, camera):
    import cv2
    bbox = mesh.get_bound_box()
    centroid = np.mean(bbox, axis=0)
    cam_pose = np.linalg.inv(camera.get_camera_pose())
    tvec = -cam_pose[0:3, 3]
    rvec = -cv2.Rodrigues(cam_pose[0:3, 0:3])[0]
    K = camera.get_intrinsics_as_K_matrix()
    dope_order = [6, 2, 1, 5, 7, 3, 0, 4]
    cuboid = [None] * 9
    for ii in range(8):
        cuboid[dope_order[ii]] = cv2.projectPoints(
            bbox[ii], rvec, tvec, K, np.array([])
        )[0][0][0]
    cuboid[8] = cv2.projectPoints(centroid, rvec, tvec, K, np.array([]))[0][0][0]
    return np.array(cuboid).tolist()

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
            "class": objects_data[ii]['class'],
            "name": objects_data[ii]['name'],
            "visibility": num_pixels,
            "projected_cuboid": projected_keypoints,
            "location": objects_data[ii]['location'],
            "quaternion_xyzw": objects_data[ii]['quaternion_xyzw']
        })
    with open(outf, "w") as write_file:
        json.dump(data, write_file, indent=4)

# ---------------- Main ---------------- #
def main():
    models_folder = "models/"
    backgrounds_folder = "backgrounds/"
    output_dir = "datasets/dope_data"
    nb_frames = 50
    width, height = 512, 512
    scale = 0.01
    min_pixels = 1

    os.makedirs(output_dir, exist_ok=True)

    bp.init()
    bp.renderer.set_output_format('PNG')
    bp.renderer.set_render_devices(desired_gpu_ids=[0])

    objects, objects_data = load_obj_models(models_folder)
    if not objects:
        print("No OBJ models found.")
        return

    # Camera: fixed position, looking at object
    cam_pose = bp.math.build_transformation_mat([0, -5, 0], [np.pi / 2, 0, 0])
    bp.camera.add_camera_pose(cam_pose)
    bp.camera.set_resolution(width, height)
    bp.camera.set_intrinsics_from_blender_params(
        lens=0.785398, lens_unit='FOV', clip_start=0.1, clip_end=1000.0
    )

    bp.renderer.enable_depth_output(activate_antialiasing=False)
    bp.renderer.set_max_amount_of_samples(50)

    for frame in tqdm(range(nb_frames), desc="Rendering DOPE frames"):
        for idx, oo in enumerate(objects):
            # Always same location in front of camera
            pose = np.eye(4)
            pose[0:3, 3] = np.array([0.0, 0.0, 0.0])  # Object at world origin
            # Random rotation each frame
            angle_x = random.uniform(0, 2*np.pi)
            angle_y = random.uniform(0, 2*np.pi)
            angle_z = random.uniform(0, 2*np.pi)
            Rx = np.array([[1, 0, 0],
                           [0, np.cos(angle_x), -np.sin(angle_x)],
                           [0, np.sin(angle_x), np.cos(angle_x)]])
            Ry = np.array([[np.cos(angle_y), 0, np.sin(angle_y)],
                           [0, 1, 0],
                           [-np.sin(angle_y), 0, np.cos(angle_y)]])
            Rz = np.array([[np.cos(angle_z), -np.sin(angle_z), 0],
                           [np.sin(angle_z), np.cos(angle_z), 0],
                           [0, 0, 1]])
            pose[0:3, 0:3] = Rz @ Ry @ Rx
            oo.set_local2world_mat(pose)
            oo.set_scale([scale] * 3)

            # Update object data
            cam_rel_pose = np.linalg.inv(bp.camera.get_camera_pose()) @ pose
            objects_data[idx]['location'] = cam_rel_pose[0:3, 3].tolist()
            q = Quaternion(matrix=cam_rel_pose[0:3, 0:3])
            objects_data[idx]['quaternion_xyzw'] = [q.x, q.y, q.z, q.w]

        segs = bp.renderer.render_segmap()
        data = bp.renderer.render()
        im = Image.fromarray(data['colors'][0])

        im.save(os.path.join(output_dir, f"{frame:06}.png"))
        write_json(os.path.join(output_dir, f"{frame:06}.json"),
                   width, height, min_pixels, bp.camera, objects, objects_data,
                   segs['class_segmaps'][0])

    print(f"{GREEN}DOPE dataset saved to {output_dir}{RESET}")

if __name__ == "__main__":
    main()
