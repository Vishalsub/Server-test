# Server-test


Requirements (Install on both PCs)
On both sender and receiver, install:

bash
Copy
Edit
pip install aiortc opencv-python aiohttp
Create folders:

bash
Copy
Edit
mkdir images


ros2 run <your_pkg> scene_add_obj1 --ros-args \
  -p planning_frame:=world \
  -p obj_frame:=obj_1 \
  -p z_reference:=top \
  -p use_obj_orientation:=true \
  -p size_x:=0.05 -p size_y:=0.05 -p size_z:=0.02 \
  -p object_id:=obj_1_box
