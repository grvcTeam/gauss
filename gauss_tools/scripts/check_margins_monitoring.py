import json
import sys
import os
import math

# Get previous directory folder
db_manager_config_directory = os.path.abspath(os.path.join(os.path.realpath(
    __file__), os.pardir, os.pardir, os.pardir)) + '/usp_nodes/db_manager/config/'
input_file_path = input(
    "An example of the file path : atlas/loss_separation\nEnter the path of your file : ")
file_to_check = db_manager_config_directory + input_file_path + '.json'
assert os.path.exists(
    file_to_check), "I did not find the file at, "+str(file_to_check)

# Number of cells
input_num_cells = int(input("Enter cells num. Default 100: ") or "100")

# Opening JSON file
f = open(file_to_check,)

# returns JSON object as a dictionary
data = json.load(f)

# Prepare lists
list_x = []
list_y = []
list_z = []
list_z.append(0)

# Get XYZ from JSON file
for i in data['operations']:
    for j in i['flight_plan']['waypoints']:
        list_x.append(j['x'])
        list_y.append(j['y'])
        list_z.append(j['z'])

# Get deltas and safety distance
deltaX = (max(list_x) - min(list_x)) / (input_num_cells - 1)
deltaY = (max(list_y) - min(list_y)) / (input_num_cells - 1)
deltaZ = (max(list_z) - min(list_z)) / (input_num_cells - 1)
# -1 because of the decimals
safety_distance = math.sqrt(deltaX**2 + deltaY**2 + deltaZ**2) - 1
# Print min max and delta for monitoring. Place these values in YAML file.
print('Minimum, maximum and suggested delta.\nCopy following lines into the YAML file.')
print('minX: ' + '{:.0f}'.format(min(list_x) - deltaX) + '\nminY: ' + '{:.0f}'.format(
    min(list_y) - deltaY) + '\nminZ: ' + '{:.0f}'.format((min(list_z))))
print('maxX: ' + '{:.0f}'.format(max(list_x) + deltaX) + '\nmaxY: ' + '{:.0f}'.format(
    max(list_y) + deltaY) + '\nmaxZ: ' + '{:.0f}'.format((max(list_z)) + deltaZ))
print('deltaX: ' + '{:.0f}'.format(deltaX) + '\ndeltaY: ' +
      '{:.0f}'.format(deltaY) + '\ndeltaZ: ' + '{:.0f}'.format(deltaZ))
# Print maximum safety distance
print('safetyDistance: ' + '{:.0f}'.format(safety_distance))
print('Safety distance can not be bigger than the diagonal of the 3D cell. The maximum is the value above.')
# Closing file
f.close()
