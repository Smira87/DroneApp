from pymavlink import mavutil
import time
import math
from math import cos, sin, atan2, sqrt, radians, degrees, asin, modf
from dronekit import connect
import json

def debug_save_log(file_path, obj):
    with open(file_path, 'w') as f:
        f.write(json.dumps(
            obj,
            sort_keys=True,
            indent=4,
            separators=(',', ': ')
        ))

def getPathLength(lat1,lng1,lat2,lng2):
    '''calculates the distance between two lat, long coordinate pairs'''
    R = 6371000 # radius of earth in m
    lat1rads = radians(lat1)
    lat2rads = radians(lat2)
    deltaLat = radians((lat2-lat1))
    deltaLng = radians((lng2-lng1))
    a = sin(deltaLat/2) * sin(deltaLat/2) + cos(lat1rads) * cos(lat2rads) * sin(deltaLng/2) * sin(deltaLng/2)
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = R * c

    return d

def getDestinationLatLong(lat,lng,azimuth,distance):
    '''returns the lat an long of destination point 
    given the start lat, long, aziuth, and distance'''
    R = 6378.1 #Radius of the Earth in km
    brng = radians(azimuth) #Bearing is degrees converted to radians.
    d = distance/1000 #Distance m converted to km

    lat1 = radians(lat) #Current dd lat point converted to radians
    lon1 = radians(lng) #Current dd long point converted to radians

    lat2 = asin( sin(lat1) * cos(d/R) + cos(lat1)* sin(d/R)* cos(brng))

    lon2 = lon1 + atan2(sin(brng) * sin(d/R)* cos(lat1), 
           cos(d/R)- sin(lat1)* sin(lat2))

    #convert back to degrees
    lat2 = degrees(lat2)
    lon2 = degrees(lon2)

    return[lat2, lon2]

res = dict()
res['fly_data'] = []
id = 1

print("Try Connect")
#vehicle = connect('COM7')
vehicle = connect('tcp:127.0.0.1:5763')

LocationGlobal = ()
LocationGlobalRelative = ()

# [m]
pos_X = 0
pos_Y = 0
pos_Z = vehicle.location.global_relative_frame.alt

# GPS
pos_Lat = vehicle.location.global_relative_frame.lat
pos_Lng = vehicle.location.global_relative_frame.lon

res['gps_calc_lat'] = pos_Lat
res['gps_calc_lng'] = pos_Lng

print("Connected")

i = 0
while (True):

	# Get all vehicle attributes (state)
	print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
	print ("Altitude (global relative frame): %s" % vehicle.location.global_relative_frame.alt)
	print ("Lat: %s" % vehicle.location.global_relative_frame.lat)
	print ("Lng: %s" % vehicle.location.global_relative_frame.lon)
	print ("Altitude (NED frame): %s" % vehicle.location.local_frame.down)
	print(" Attitude: %s" % vehicle.attitude)
	print(" Velocity: %s" % vehicle.velocity)
	print(" GPS: %s" % vehicle.gps_0)
	print(" Gimbal status: %s" % vehicle.gimbal)
	print(" Battery: %s" % vehicle.battery)
	print(" EKF OK?: %s" % vehicle.ekf_ok)
	print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
	print(" Is Armable?: %s" % vehicle.is_armable)
	print(" System status: %s" % vehicle.system_status.state)
	print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
	print(" Airspeed: %s" % vehicle.airspeed)    # settable
	print(" Mode: %s" % vehicle.mode.name)    # settable
	print(" Armed: %s" % vehicle.armed)    # settable
	print(" Yaw: %s" % vehicle.attitude.yaw)
	
	if (str(vehicle.attitude.yaw) == 'None'):
		vehicle.attitude.yaw = 0

	fly_data = dict()
	fly_data['id'] = id
	fly_data['time'] = round(time.time())
	fly_data['time_view'] = time.strftime("%d.%m.%Y %H:%M:%S")
	fly_data['time_full'] = time.time()
	fly_data['mode'] = vehicle.mode.name
	fly_data['yaw'] = vehicle.attitude.yaw
	fly_data['lat'] = vehicle.location.global_relative_frame.lat
	fly_data['lng'] = vehicle.location.global_relative_frame.lon

	print(fly_data['yaw'])
	yaw_degrees = fly_data['yaw'] * 180.0 / 3.14 # conversion to degrees
	if( yaw_degrees < 0 ): yaw_degrees += 360.0 # convert negative to positive angles
	print(" Yav degrees: %s" % yaw_degrees)    # settable
	fly_data['yaw_degrees'] = yaw_degrees
	fly_data['velocity_x'] = vehicle.velocity[0]
	fly_data['velocity_y'] = vehicle.velocity[1]
	fly_data['velocity_z'] = vehicle.velocity[2]
	fly_data['alt'] = vehicle.location.global_relative_frame.alt
	
	if (id > 1):
		print(prev_fly_data)
		delta_yaw_degrees = fly_data['yaw_degrees'] - prev_fly_data['yaw_degrees']
		#print(delta_yaw_degrees)
		fly_data['delta_yaw_degrees'] = delta_yaw_degrees
		fly_data['Rotation_state'] = 'NONE'
		if (fly_data['delta_yaw_degrees'] > 5): fly_data['Rotation_state'] = 'RIGHT'
		if (fly_data['delta_yaw_degrees'] < -5): fly_data['Rotation_state'] = 'LEFT'

		delta_alt_degrees = fly_data['alt'] - prev_fly_data['alt']
		fly_data['delta_alt_degrees'] = delta_alt_degrees
		fly_data['Lift_state'] = 'NONE'
		if (fly_data['delta_alt_degrees'] > 0.2): fly_data['Lift_state'] = 'UP'
		if (fly_data['delta_alt_degrees'] < -0.2): fly_data['Lift_state'] = 'DOWN'
		print('===================================')
		print('Rotation: '+str(fly_data['Rotation_state'])+' Lift: '+str(fly_data['Lift_state']))
		print('===================================')
		fly_data['d_lat'] = prev_fly_data['lat'] - fly_data['lat']
		fly_data['d_lng'] = prev_fly_data['lng'] - fly_data['lng']
		print('d_Lat: '+str(fly_data['d_lat']))
		print('d_Lng: '+str(fly_data['d_lng']))
		print('===================================')
		print('v_X: '+str(fly_data['velocity_x']))
		print('v_Y: '+str(fly_data['velocity_y']))
		print('===================================')
		pos_X += fly_data['velocity_x']/5
		pos_Y += fly_data['velocity_y']/5

		fly_data['d_X'] = fly_data['velocity_x']/5
		fly_data['d_Y'] = fly_data['velocity_y']/5
		
		fly_data['pos_X'] = pos_X
		fly_data['pos_Y'] = pos_Y
		fly_data['pos_Z'] = fly_data['alt']

		print('pos_X: '+str(fly_data['pos_X']))
		print('pos_Y: '+str(fly_data['pos_Y']))
		print('pos_Z: '+str(fly_data['pos_Z']))
		print('Yaw: '+str(fly_data['yaw_degrees'])+" ("+str(fly_data['yaw'])+")")
		print('===================================')

		fly_data['d_distance'] = math.sqrt(fly_data['d_X'] ** 2 + fly_data['d_Y'] ** 2)
		print('d_X: '+str(fly_data['d_X']))
		print('d_Y: '+str(fly_data['d_Y']))
		print('d_distance: '+str(fly_data['d_distance']))
		print('===================================')
		fly_data['gps_calc'] = getDestinationLatLong(res['gps_calc_lat'], res['gps_calc_lng'], fly_data['yaw_degrees'], fly_data['d_distance'])
		res['gps_calc_lat'] = fly_data['gps_calc'][0]
		res['gps_calc_lng'] = fly_data['gps_calc'][1]
		print('Lat_calc: '+str(res['gps_calc_lat']))
		print('Lng_calc: '+str(res['gps_calc_lng']))
		res['gps_calc_lat_diff'] = res['gps_calc_lat']-fly_data['lat']
		res['gps_calc_lng_diff'] = res['gps_calc_lng']-fly_data['lng']
		print('Lat_calc_diff: '+str(res['gps_calc_lat_diff']))
		print('Lng_calc_diff: '+str(res['gps_calc_lng_diff']))
		res['gps_dist_diff'] = getPathLength(res['gps_calc_lat'], res['gps_calc_lng'], fly_data['lat'], fly_data['lng'])
		print('gps_dist_diff: '+str(res['gps_dist_diff']))
		print('===================================')
		print('pos_Lat: '+str(fly_data['lat']))
		print('pos_Lng: '+str(fly_data['lng']))
		print('GMaps: https://www.google.com/maps?q='+str(fly_data['lat'])+','+str(fly_data['lng'])+'&t=k')
		print('===================================')
		if (fly_data['velocity_y'] != 0 and fly_data['d_lat'] != 0):
			print('k_X: '+str(fly_data['velocity_y']/fly_data['d_lat']))
		if (fly_data['velocity_x'] != 0 and fly_data['d_lng'] != 0):
			print('k_Y: '+str(fly_data['velocity_x']/fly_data['d_lng']))
		print('===================================')

	debug_save_log('fly_data_now.txt', fly_data)
	res['fly_data'].append(fly_data)
	debug_save_log('fly_log.txt', res)
	print(fly_data)
	
	id += 1
	prev_fly_data = fly_data
	time.sleep(0.2)

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()



print("Completed")

