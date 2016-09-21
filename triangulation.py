import roslib
from std_msgs.msg import String
import sensor_msgs.msg
from math import *

# Threshold for determining which data points correspond to beacons
global THRESHOLD
THRESHOLD = 19

# These will later be initialized as 2D arrays
global scanData
global scanData1

# Beacon positions
global beaconX
beaconX = [ 0, 0, 0 ]
global beaconY
beaconY = [ 0, .1, .2 ]

# Used in ensuring two scans are used for processing
global scanCount
scanCount = 0

# Number of scans obtained since the program was started
global numberScans
numberScans = 0

global missedScans
missedScans = 0

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
	print( scanData[0] )
	print( scanData[1] )

# Locate the beacons in the scan data
# Looking for clumped together points in the data
def findBeacons( scanData, inc ):
	print( "BEACONS" )
	beaconsFound = 0
	started = 0

	# Make a 2D array to contain distance and angles to each beacon
	# beaconData[0] = ANGLES
	# beaconData[1] = DISTANCES
	global beaconData
	beaconData = []
	beaconData.append( [] )
	beaconData.append( [] )

	# Find the average difference in angle from point to point
	avgDiffTheta = 0
	for i in range( 1, len( scanData[0] ) ):
		avgDiffTheta = avgDiffTheta + ( scanData[0][i] - scanData[0][i-1] )
	avgDiffTheta = avgDiffTheta / ( len( scanData[0] ) - 1 )

	# Loop through the length of the scan data
	for i in range( len( scanData[0] ) - 1 ):
		# Only keep looking for beacons if there aren't the three needed
		if( beaconsFound < 3 ):
			# Check to see if the angle between two points is less than the average
			# Indicates that these are two points on one beacon rather than on two beacons
			if( abs( scanData[0][i] - scanData[0][i + 1] ) < avgDiffTheta and scanData[1][i] < 2 ):
				if( started == 0 ):
					beaconStart = i
					started = 1
			# If the angle between points becomes larger than the average, a beacon has been found or there was erroneous data
			else:
				# If a beacon has already been started, then the larger-than-average distance indicates that the end of the beacon has been found
				if( started == 1 ):
					beaconEnd = i
					started = 0
					# The threshold value for the number of points needed to make a beaco is currently set to 10
					# Set the distance and angle of the beacon in the 2D array of beacon data
					if( ( beaconEnd - beaconStart ) >= 10 ):
						beaconData[0].append( scanData[0][ beaconStart + int( floor( ( beaconEnd - beaconStart ) / 2 ) ) ] )
						beaconData[1].append( scanData[1][ beaconStart + int( floor( ( beaconEnd - beaconStart ) / 2 ) ) ] )
						beaconsFound = beaconsFound + 1
	# Print some outputs
	print( avgDiffTheta )
	print( beaconData[0] )
	print( beaconData[1] )

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

		print( options )

		d1 = sqrt( pow( options[0][0] - beaconX[2], 2 ) + pow( options[0][1] - beaconY[2], 2 ) )
		d2 = sqrt( pow( options[1][0] - beaconX[2], 2 ) + pow( options[1][1] - beaconY[2], 2 ) )
		if( abs( d1 - beaconData[1][2] ) < abs( d2 - beaconData[1][2] ) ):
			coordinates = options[0]
		else:
			coordinates = options[1]

		print( "POSITION" )
		print( coordinates )
		print( "" )
	else:
		print( "NOT ENOUGH BEACONS FOUND" )
		print( "" )
		global missedScans
		missedScans = missedScans + 1

def calcAngle():	
	print( "ANGLE" )

# Set up listener to detect a new LiDAR scan and execute the callback function
def listener():
	rospy.init_node( 'listener', anonymous=True )
	rospy.Subscriber( "/scan", sensor_msgs.msg.LaserScan, callback )
	rospy.spin()

if __name__ == '__main__':
	listener()
