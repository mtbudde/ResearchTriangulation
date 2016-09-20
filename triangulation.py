import rospy
import roslib
from std_msgs.msg import String
import sensor_msgs.msg
from math import *

global THRESHOLD
THRESHOLD = 31

global scanData
global scanData1

global beaconX
beaconX = [ 0, 0, 0 ]
global beaconY
beaconY = [ 0, .1, .2 ]
global scanCount
scanCount = 0
global numberScans
numberScans = 0

def callback(data):
	startAngle = data.angle_min
	endAngle = data.angle_max
	angleInc = data.angle_increment
	ranges = data.ranges
	intensities = data.intensities
	global scanCount
	global numberScans
	if( scanCount == 0 ):
		firstScan( startAngle, endAngle, angleInc, ranges, intensities )
	if( scanCount == 1 ):
		processScan( startAngle, endAngle, angleInc, ranges, intensities, scanData1 )
		findBeacons( scanData )
		calcPosition( beaconX, beaconY, beaconData )
		calcAngle( )
		numberScans = numberScans + 1
		print( numberScans )
	scanCount = ( scanCount + 1 ) % 2

def firstScan( start, end, inc, ranges, intensities ):
	global scanData1
	scanData1 = []
	scanData1.append( [] )
	scanData1.append( [] )
	for i in range( len( intensities ) ):
		if intensities[i] > THRESHOLD:
			scanData1[0].append( start + ( i * inc ) )
			scanData1[1].append( ranges[i] )

def processScan( start, end, inc, ranges, intensities, scanData1 ):
	print( "PROCESS" )
	global scanData
	scanData = []
	scanData.append( [] )
	scanData.append( [] )
	for i in range( len( intensities ) ):
		if intensities[i] > THRESHOLD:
			scanData[0].append( start + ( i * inc ) )
			scanData[1].append( ranges[i] )

	k = 0
	l = 0
	while l < len( scanData[0] ) and k < len( scanData1[0] ):
		if( scanData1[0][k] < scanData[0][l] ):
			scanData[0].insert( l, scanData1[0][k] )
			scanData[1].insert( l, scanData1[1][k] )
			k = k + 1
		else:
			l = l + 1

	print( scanData[0] )
	print( scanData[1] )

def findBeacons( scanData ):
	print( "BEACONS" )
	beaconsFound = 0
	started = 0
	global beaconData
	beaconData = []
	beaconData.append( [] )
	beaconData.append( [] )
	for i in range( len( scanData[0] ) - 1 ):
		if( beaconsFound < 3 ):
			if( abs( scanData[0][i] - scanData[0][i + 1] ) < .05 ):
				if( started == 0 ):
					beaconStart = i
					started = 1
			else:
				if( started == 1 ):
					beaconEnd = i
					started = 0
					if( ( beaconEnd - beaconStart ) >= 10 ):
						beaconData[0].append( scanData[0][ beaconStart + int( floor( ( beaconEnd - beaconStart ) / 2 ) ) ] )
						beaconData[1].append( scanData[1][ beaconStart + int( floor( ( beaconEnd - beaconStart ) / 2 ) ) ] )
						beaconsFound = beaconsFound + 1

#	beaconData[0] = [ 3.938, 3.593, 3.145 ]
#	beaconData[1] = [ .284, .2225, .2049 ]
	print( beaconData[0] )
	print( beaconData[1] )

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

def calcAngle():	
	print( "ANGLE" )

def listener():
	rospy.init_node( 'listener', anonymous=True )
	rospy.Subscriber( "/scan", sensor_msgs.msg.LaserScan, callback )
	rospy.spin()

if __name__ == '__main__':
	listener()
