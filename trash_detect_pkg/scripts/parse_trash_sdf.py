import xml.etree.ElementTree as ET
import os
import csv

# Path to your world file
world_file = os.path.expanduser('~/catkin_ws/src/heron_simulator/heron_gazebo/worlds/ocean_surface_trash.world')
output_csv = os.path.expanduser('~/catkin_ws/trash_ground_truth.csv')

# Parse XML
tree = ET.parse(world_file)
root = tree.getroot()

# Namespace fix if required
ns = {'sdf': 'http://sdformat.org/schemas/root.xsd'}

# Extract all trash model poses
trash_locations = []

for model in root.findall('.//model'):
    name = model.attrib.get('name', '')
    if 'floating_trash' in name:
        pose_elem = model.find('pose')
        if pose_elem is not None:
            pose = pose_elem.text.strip().split()
            x, y = float(pose[0]), float(pose[1])
            trash_locations.append((x, y))

# Save to CSV
with open(output_csv, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(['x', 'y'])  # header
    writer.writerows(trash_locations)

print(f"✅ Extracted {len(trash_locations)} trash locations to {output_csv}")

