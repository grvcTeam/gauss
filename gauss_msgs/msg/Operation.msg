uint8 uav_id
string icao_address
uint8 frame
float64 autonomy			# distance (meters)
uint8 priority		
int32 current_wp
WaypointList flight_plan
float64 dT				# time (seconds) between each pair of waypoints
float64 time_tracked			# time (seconds) saving tracks
WaypointList track			
float64 time_horizon			# time (seconds)
WaypointList estimated_trajectory
float64 flight_geometry			# meters
float64 operational_volume		# meters
string conop

uint8 FRAME_ROTOR = 0
uint8 FRAME_FIXEDWING = 1
