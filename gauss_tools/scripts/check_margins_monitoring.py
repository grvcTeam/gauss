import json
import sys
import os

# Get previous directory folder
db_manager_config_directory = os.path.abspath(os.path.join(os.path.realpath(__file__), os.pardir, os.pardir, os.pardir)) + '/usp_nodes/db_manager/config/'

user_input = input("Enter the path of your file. Example: atlas/loss_separation\nInput: ")

file_to_check = db_manager_config_directory + user_input + '.json'
assert os.path.exists(file_to_check), "I did not find the file at, "+str(file_to_check)

# Opening JSON file
f = open(file_to_check,)
  
# returns JSON object as a dictionary
data = json.load(f)


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

# Number of cells
num_cells = 50

# Print min max and delta for monitoring. Place these values in YAML file.
print('Minimum, maximum and suggested delta. Number of cells of the monitoring matrix: ' + str(num_cells) + '.')
print('X -> [' + '{:.2f}'.format(min(list_x)) + ', ' + '{:.2f}'.format(max(list_x)) + ']' + ' (' + '{:.2f}'.format((max(list_x) - min(list_x)) / num_cells) + ')' )
print('Y -> [' + '{:.2f}'.format(min(list_y)) + ', ' + '{:.2f}'.format(max(list_y)) + ']' + ' (' + '{:.2f}'.format((max(list_y) - min(list_y)) / num_cells) + ')' )
print('Z -> [' + '{:.2f}'.format(min(list_z)) + ', ' + '{:.2f}'.format(max(list_z)) + ']' + ' (' + '{:.2f}'.format((max(list_z) - min(list_z)) / num_cells) + ')' )

# Closing file
f.close()