#!/usr/bin/env python3
import blenderproc as bp
import numpy as np
import os
import random
from PIL import Image
from tqdm import tqdm
from pyquaternion import Quaternion
import json

# ---------- USER CONFIG ----------
blend_file = "scene.blend"   # Path to your .blend file
object_name = "MyObject"     # Exact name of object in Blender Outliner
output_dir = "datasets/dope_data"
nb_frames = 50
width, height = 512, 512
min_pixels = 1
scale_factor = 1.0  # Keep 1.0 if your object size is correct
# ---------------------------------

GREEN = "\033[1;32m"
RED = "\033[1;31m"
RESET = "\033[0m"

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
        cuboid[dope_order[ii]] = cv2.projectPoints(bbox[ii], rvec, tvec, K, np.array([]))[0][0][0]
    cuboid[8] = cv2.projectPoints(centroid, rvec, tvec, K, np.array([]))[0][0][0]
    return np.array(cuboid).tolist()

def write_json(outf, width, height, min_pixels, camera, obj, obj_data, seg_map):
    cam_xform = camera.get_camera_pose()
    eye = -cam_xform[0:3, 3]
    at = -cam_xform[0:3, 2]
    up = cam_xform[0:3, 0]
    K = camera.get_intrinsics_as_K_matrix()
    num_pixels = int(np.sum((seg_map == obj_data['id'])))
    if num_pixels < min_pixels:
        return
    projected_keypoints = get_cuboid_image_space(obj, camera)
    data = {
        "camera_data": {
            "width": width,
            "height": height,
            "camera_look_at": {"eye": eye.tolist(), "at": at.tolist(), "up": up.tolist()},
            "intrinsics": {"fx": K[0][0], "fy": K[1][1], "cx": K[2][0], "cy": K[2][1]}
        },
        "objects": [{
            "class": obj_data['class'],
            "name": obj_data['name'],
            "visibility": num_pixels,
            "projected_cuboid": projected_keypoints,
            "location": obj_data['location'],
            "quaternion_xyzw": obj_data['quaternion_xyzw']
        }]
    }
    with open(outf, "w") as write_file:
        json.dump(data, write_file, indent=4)

def main():
    os.makedirs(output_dir, exist_ok=True)
    bp.init()
    bp.loader.load_blend(blend_file)
    bp.renderer.set_output_format('PNG')
    bp.renderer.set_render_devices(desired_gpu_ids=[0])
    bp.renderer.set_max_amount_of_samples(50)

    # Find object in scene
    scene_objects = bp.object.get_all_mesh_objects()
    target_obj = None
    for o in scene_objects:
        if o.get_name() == object_name:
            target_obj = o
            break
    if target_obj is None:
        print(f"{RED}Object '{object_name}' not found in scene!{RESET}")
        return

    target_obj.set_scale([scale_factor] * 3)
    obj_data = {
        "class": object_name,
        "name": object_name + "_000",
        "id": 1
    }

    for frame in tqdm(range(nb_frames), desc="Rendering DOPE frames"):
        pose = target_obj.get_local2world_mat()
        # Random rotation
        angle_x = random.uniform(0, 2*np.pi)
        angle_y = random.uniform(0, 2*np.pi)
        angle_z = random.uniform(0, 2*np.pi)
        Rx = np.array([[1, 0, 0], [0, np.cos(angle_x), -np.sin(angle_x)], [0, np.sin(angle_x), np.cos(angle_x)]])
        Ry = np.array([[np.cos(angle_y), 0, np.sin(angle_y)], [0, 1, 0], [-np.sin(angle_y), 0, np.cos(angle_y)]])
        Rz = np.array([[np.cos(angle_z), -np.sin(angle_z), 0], [np.sin(angle_z), np.cos(angle_z), 0], [0, 0, 1]])
        pose[0:3, 0:3] = Rz @ Ry @ Rx
        target_obj.set_local2world_mat(pose)

        # Update DOPE pose info
        cam_rel_pose = np.linalg.inv(bp.camera.get_camera_pose()) @ pose
        obj_data['location'] = cam_rel_pose[0:3, 3].tolist()
        q = Quaternion(matrix=cam_rel_pose[0:3, 0:3])
        obj_data['quaternion_xyzw'] = [q.x, q.y, q.z, q.w]

        segs = bp.renderer.render_segmap()
        data = bp.renderer.render()
        im = Image.fromarray(data['colors'][0])
        im.save(os.path.join(output_dir, f"{frame:06}.png"))
        write_json(os.path.join(output_dir, f"{frame:06}.json"), width, height, min_pixels, bp.camera, target_obj, obj_data, segs['class_segmaps'][0])

    print(f"{GREEN}DOPE dataset saved in {output_dir}{RESET}")

if __name__ == "__main__":
    main()
