import rospy
import roslib
from std_msgs.msg import String
import sensor_msgs.msg
from math import *

# Threshold for determining which data points correspond to beacons
global THRESHOLD
THRESHOLD = 12

# These will later be initialized as 2D arrays
global scanData
global scanData1

# Beacon positions
global beaconX
beaconX = [ 0, 0, 0 ]
global beaconY
beaconY = [ .5, .3, .1 ]

# Used in ensuring two scans are used for processing
global scanCount
scanCount = 0

# Number of scans obtained since the program was started
global numberScans
numberScans = 0

global missedScans
missedScans = 0

global coordinates
#coordinates = []

# Callback function for a LiDAR scan
def callback(data):
	# Pull all data from the sensor message into local variables
	startAngle = data.angle_min
	endAngle = data.angle_max
	angleInc = data.angle_increment
	ranges = data.ranges
	intensities = data.intensities

	# Use these two global variables
	global scanCount
	global numberScans
	# If there is only one scan available, store in memory
	if( scanCount == 0 ):
		firstScan( startAngle, endAngle, angleInc, ranges, intensities )
	if( scanCount == 1 ):
		secondScan( startAngle, endAngle, angleInc, ranges, intensities, scanData1 )
	# If two scans have been obtained, combine them and attempt to find the position of the device
	if( scanCount == 2 ):
		processScan( startAngle, endAngle, angleInc, ranges, intensities, scanData1 )
		findBeacons( scanData, angleInc )
		calcPosition( beaconX, beaconY, beaconData )
		calcAngle( )
		numberScans = numberScans + 1
		print( numberScans )
		print( missedScans )
	scanCount = ( scanCount + 1 ) % 3 # Roll the scan count around
#	print coordinates
#	return coordinates

# Function to process the first scan - using two scans to get a suitable number of high-quality data points
def firstScan( start, end, inc, ranges, intensities ):
	# Create a global 2D array to store the first scan data
	global scanData1
	scanData1 = []
	scanData1.append( [] )
	scanData1.append( [] )
	# Add all scan data to the global 2D array
	for i in range( len( intensities ) ):
		if intensities[i] > THRESHOLD:
			scanData1[0].append( start + ( i * inc ) )
			scanData1[1].append( ranges[i] )

# Function to process the second scan - using three scans
def secondScan( start, end, inc, ranges, intensities, scanData1 ):
	scanTemp = []
	scanTemp.append( [] )
	scanTemp.append( [] )
	# Temporarily store the scan data
	for i in range( len( intensities ) ):
		if intensities[i] > THRESHOLD:
			scanTemp[0].append( start + ( i * inc ) )
			scanTemp[1].append( ranges[i] )
	
	# Insert the temporary scan into the global array
	k = 0
	l = 0
	while l < len( scanData1[0] ) and k < len( scanTemp[0] ):
		if( scanTemp[0][k] < scanData1[0][1] ):
			scanData1[0].insert( 1, scanTemp[0][k] )
			scanData1[1].insert( 1, scanTemp[1][k] )
			k = k + 1
		else:
			l = l + 1

# Function to process both scans together
def processScan( start, end, inc, ranges, intensities, scanData1 ):
	# Create a global 2D array to store both scans
	global scanData
	scanData = []
	scanData.append( [] )
	scanData.append( [] )
	# Add the second scan in to the new array
	for i in range( len( intensities ) ):
		if intensities[i] > THRESHOLD:
			scanData[0].append( start + ( i * inc ) )
			scanData[1].append( ranges[i] )

	# Insert the first scan into the master array point-by-point in the appropriate positions
	k = 0
	l = 0
	while l < len( scanData[0] ) and k < len( scanData1[0] ):
		if( scanData1[0][k] < scanData[0][l] ):
			scanData[0].insert( l, scanData1[0][k] )
			scanData[1].insert( l, scanData1[1][k] )
			k = k + 1
		else:
			l = l + 1

	# Print some output
	#print( scanData[0] )
	#print( scanData[1] )

# Locate the beacons in the scan data
# Looking for clumped together points in the data
def findBeacons( scanData, inc ):
	# State and Counter Variable Setup
	i = 1
	j = 0
	beacCount = 0
	higher = 0
	started = 0
	beginning = 0
	end = 0;
	index = 0;
	# End Setup
	
	# Matrix Setup
	# peaks = [ang][dist][count]
	peaks = []
	peaks.append( [] )
	peaks.append( [] )
	peaks.append( [] )
	
	# beaconData = [ang][dist]
	global beaconData
	beaconData = []
	beaconData.append( [] )
	beaconData.append( [] )
	# End Setup
	
	# Find the average difference in angle from point to point
	avgDiffTheta = 0
	avgDist = 0
	for i in range( 1, len( scanData[0] ) ):
		avgDiffTheta = avgDiffTheta + ( scanData[0][i] - scanData[0][i-1] )
		avgDist = avgDist + scanData[1][i]
	avgDiffTheta = avgDiffTheta / ( len( scanData[0] ) - 1 )
	avgDist = avgDist / ( len( scanData[1] ) - 1 ) / 2
	# Average distance is found
	# Find all peaks in the scan data
	i = 1
	while( i < len( scanData[0] ) ):
		if( ( scanData[0][i] - scanData[0][i-1] ) < avgDiffTheta ):
			if( started != 1 ):
				started = 1
				beginning = i
		else:
			if( started == 1 ):
				end = i - 1
				started = 0
				index = int(( end - beginning ) / 2) + beginning
				peaks[0].append( scanData[0][index] )
				peaks[1].append( scanData[1][index] )
				peaks[2].append( end - beginning )
		i = i + 1
	# All peaks found
	print( "Peaks Found:" )
	print( peaks[0] )
	print( peaks[1] )
	print( peaks[2] )
	print()
	# Find the three highest peaks in the list of peaks and assign them to the beacons
	peaks = zip( *peaks )
	peaks = sorted( peaks, key=lambda l:l[2] )
	peaks = zip( *peaks )
	
	beaconData[0].append( peaks[0][0] )
	beaconData[1].append( peaks[1][0] )
	beaconData[0].append( peaks[0][1] )
	beaconData[1].append( peaks[1][1] )
	beaconData[0].append( peaks[0][2] )
	beaconData[1].append( peaks[1][2] )
	# All data assigned

# For explanation of workings, see http://jwilson.coe.uga.edu/EMAT6680Fa05/Schultz/6690/Barn_GPS/Barn_GPS.html
# This function corresponds to the Simulink module created by Matthew Budde
def calcPosition( beaconX, beaconY, beaconData ):
	if( len( beaconData[0] ) == 3 ):
		beaconDataTranslated = []
		beaconDataTranslated.append( [] )
		beaconDataTranslated.append( [] )
		for i in range( len( beaconData[0] ) ):
			beaconDataTranslated[0].append( beaconX[i] - beaconX[0] )
			beaconDataTranslated[0][i] = float( beaconDataTranslated[0][i] )
			beaconDataTranslated[1].append( beaconY[i] - beaconY[0] )
			beaconDataTranslated[1][i] = float( beaconDataTranslated[1][i] )

		R1 = sqrt( pow( beaconX[1] - beaconX[0], 2 ) + pow( beaconY[1] - beaconY[0], 2 ) )
		R2 = sqrt( pow( beaconX[2] - beaconX[0], 2 ) + pow( beaconY[2] - beaconY[0], 2 ) )
		theta = atan2( beaconDataTranslated[1][1], beaconDataTranslated[0][1] )
		phi = atan2( beaconDataTranslated[1][2], beaconDataTranslated[0][2] )

		polarR = [0, float( R1 ), float( R2 )]
		polarT = [0, 0, float( phi - theta )]

		rotatedX = [0, polarR[1], ( polarR[2] * cos( polarT[2] ) )]
		rotatedY = [0, 0, ( polarR[2] * sin( polarT[2] ) )]
	
		rotatedYSolution = []
		rotatedXSolution = float( ( ( pow( beaconData[1][0], 2) - pow( beaconData[1][1], 2 ) ) + pow( rotatedX[1], 2 ) ) / ( 2 * rotatedX[1] ) )
		rotatedYSolution.append( float( sqrt( abs( pow( beaconData[1][0], 2 ) - pow( rotatedXSolution, 2 ) ) ) ) )
		rotatedYSolution.append( float( rotatedYSolution[0] * -1 ) )
	
		polarRotatedT = []
		polarRotatedR = sqrt( pow( rotatedXSolution, 2 ) + pow( rotatedYSolution[0], 2 ) )
		polarRotatedT.append( atan2( rotatedYSolution[0], rotatedXSolution ) )
		polarRotatedT.append( atan2( rotatedYSolution[1], rotatedXSolution ) )

		unrotatedT = []
		unrotatedR = polarRotatedR
		unrotatedT.append( theta + polarRotatedT[0] )
		unrotatedT.append( theta + polarRotatedT[1] )

		rectangularTranslatedX = []
		rectangularTranslatedY = []
		rectangularTranslatedX.append( unrotatedR * cos( unrotatedT[0] ) )
		rectangularTranslatedX.append( unrotatedR * cos( unrotatedT[1] ) )
		rectangularTranslatedY.append( unrotatedR * sin( unrotatedT[0] ) )
		rectangularTranslatedY.append( unrotatedR * sin( unrotatedT[1] ) )

		options = []
		options.append( [] )
		options.append( [] )
		options[0].append( beaconX[0] + rectangularTranslatedX[0] )
		options[0].append( beaconY[0] + rectangularTranslatedY[0] )
		options[1].append( beaconX[0] + rectangularTranslatedX[1] )
		options[1].append( beaconY[0] + rectangularTranslatedY[1] )

#		print( options )

		d1 = sqrt( pow( options[0][0] - beaconX[2], 2 ) + pow( options[0][1] - beaconY[2], 2 ) )
		d2 = sqrt( pow( options[1][0] - beaconX[2], 2 ) + pow( options[1][1] - beaconY[2], 2 ) )
		global coordinates
		if( abs( d1 - beaconData[1][2] ) < abs( d2 - beaconData[1][2] ) ):
			coordinates = options[0]
		else:
			coordinates = options[1]
		coordinates[0] = abs( coordinates[0] )
		coordinates[1] = abs( coordinates[1] )

#		print( "POSITION" )
		print( coordinates )
#		print( "" )
#		print( options[0] )
#		print( options[1] )
	else:
#		print( "NOT ENOUGH BEACONS FOUND" )
		print( "" )
		global missedScans
		missedScans = missedScans + 1
		

def calcAngle():
	truckAngle = 0
	divisor = 3
	global coordinates
	print( coordinates[0] )
	print( coordinates[1] )
	if( len( beaconData[0] ) == 3 ):
		for i in range(0, 2):
			base = abs( atan( ( coordinates[1] - beaconY[i] ) / ( coordinates[0] - beaconX[i] ) ) )
			xCond = coordinates[0] - beaconX[i]
			yCond = coordinates[1] - beaconY[i]
			if xCond > 0 and yCond > 0:
				truckAngle = truckAngle + ( pi - beaconData[0][i] + base )
			elif xCond <=0 and yCond > 0:
				truckAngle = truckAngle + ( 2 * pi - beaconData[0][i] - base )
			elif xCond <=0 and yCond <= 0:
				truckAngle = truckAngle + ( 2 * pi - beaconData[0][i] + base )
			elif xCond > 0 and yCond <= 0:
				truckAngle = truckAngle + ( pi - beaconData[0][i] - base )
			else:
				divisor = divisor - 1

	truckAngle = truckAngle / divisor
	print( "ANGLE" )
	print( truckAngle )

# Set up listener to detect a new LiDAR scan and execute the callback function
def listener():
	rospy.init_node( 'listener', anonymous=True )
	rospy.Subscriber( "/scan", sensor_msgs.msg.LaserScan, callback )
	rospy.spin()
	return coordinates

if __name__ == '__main__':
	listener()
