# https://blender.stackexchange.com/questions/34537/how-to-batch-convert-between-file-formats
import bpy
import os

path="/home/ana/ros/plum/src/m2020-urdf-models/rover/meshes"

for root, dirs, files in os.walk(path):
  for f in files:
    if f.endswith('.gltf'):
      gltf_file = os.path.join(path, f)
      obj_file = os.path.splitext(gltf_file)[0] + ".obj"
      
      bpy.ops.object.select_all(action='SELECT')
      bpy.ops.object.delete()
      
      bpy.ops.import_scene.gltf(filepath=gltf_file)
      bpy.ops.object.select_all(action='SELECT')
      bpy.ops.export_scene.obj(filepath=obj_file)
      
