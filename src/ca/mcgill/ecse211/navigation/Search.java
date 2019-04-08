package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.NavigationController;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.project.Project2;
import ca.mcgill.ecse211.project.UltrasonicController;
import ca.mcgill.ecse211.project.WifiInfo;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.controller.UltrasonicPoller;

/**
 * Search class, searches for cans in the search zone
 * 
 * @author Sumail
 *
 */
public class Search implements NavigationController {
	
	/**
	 * create a constant holding max amount of cans
	 */
	
	private static final int MAXNUMBERCANS = 2;
	/**
	 * create all robot motors, odometer instance
	 */
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor clawMotor;
	private EV3MediumRegulatedMotor sensorMotor;
	private Odometer odometer;
	private boolean isNavigating = false;
	
	/**
	 * angle status before search
	 * distance between two front wheels 
	 */
	private double anglebeforesearch=0;
	public double TRACK;
	/**
	 * Current position of the robot [0] = X corr (cm), [1] = Y corr (cm), [2] = theta (deg)
	 * Instantiate ultrasonic sensor
	 */
	private static double currentPosition[] = new double[3];
	private SensorModes usSensor; 
	
	/**
	 * keeps track of number of cans in storage area
	 * keeps track of number of heavy cans
	 */
	private int numcans = 0; 
	private int numheavy = 0; 
	
	/**
	 * float array for ultrasonic data
	 * fetch samples from ultrasonic sensor
	 */
	private float[] usData;
	private SampleProvider usDistance;
	
	/**
	 * specific distance of x and y from x=0 and y=0
	 */
	private double xcoor = 0;
	private double ycoor = 0;

	/**
	 * WiFi class parameters to decide the search zone
	 * Including coordinates for lower left and upper right point
	 * Corresponding to related paramaters fetch from WIFI GUI
	 */
	private int SZR_UR_x=WifiInfo.Search_UR_x; 
    private int SZR_UR_y=WifiInfo.Search_UR_y; 
	private int SZR_LL_x=WifiInfo.Search_LL_x; 
	private int SZR_LL_y=WifiInfo.Search_LL_y;
	 
	/**
	 * Create instance for two light sensors 
	 * used for correction 
	 */
	private static LightSensorController leftLS;
	private static LightSensorController rightLS;

	/**
	 * color calibration class used for color identification 
	 */
	private ColorCalibration cc;

	/**
	 * Constructor for search class
	 */
	public Search(EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor, Odometer odometer,
			SampleProvider usDistance, LightSensorController leftLS, LightSensorController rightLS,
			EV3LargeRegulatedMotor clawMotor, EV3MediumRegulatedMotor sensorMotor, double track) {
		// Reset the motors
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.setAcceleration(600);
		rightMotor.setAcceleration(600);
		leftMotor.setSpeed(250);
		rightMotor.setSpeed(250);
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.leftLS = leftLS;
		this.rightLS = rightLS;
		this.sensorMotor = sensorMotor;
		this.clawMotor = clawMotor;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];
		this.TRACK = track;

	}

	/**
	 * Main method to search for cans in the search field
	 * 
	 */
	public void searchcans() {

		//Sequence of three beeps 
		Sound.twoBeeps();
		Sound.beep();
		// calculate search area size
		int column = SZR_UR_x - SZR_LL_x;
		int row = SZR_UR_y - SZR_LL_y;
		//record theta when robot starts 
		anglebeforesearch=odometer.getTheta();
		//set theta to 0, ensure search method working
		odometer.setTheta(0);
		// travel along the length of the search field, tile by tile
		for (int i = 0; i < column; i++) {
			
			if (numcans == MAXNUMBERCANS) { // stop searching once we have collected two cans
				return;
			}
			
			//if the column count is odd
			if (i % 2 != 0) {
				for (int j = row; j > -1; j--) {
					if (j != 0) {
						scanTile(i);
						if (numcans == MAXNUMBERCANS) { // stop searching once we have collected two cans
							return;
						}
						// otherwise continue searching
						moveforward();
						ycoor = ycoor - 30.48;
					} else { // once we reach the end of a row, correct position and turn left to next row
						correctPosition();
						turnLeft(90);
					}
				}
				//move to next block and make a turn if reach to the edge 
				moveforward();
				correctPosition();
				turnLeft(90);
				//increase x coordinate by one block distance
				xcoor = xcoor + 30.48;
			} 
			
			//when column count is even
			else {
				for (int j = 0; j < row + 1; j++) {
					if (j != row) {
						scanTile(i);
						if (numcans == MAXNUMBERCANS)       // stop searching once we have collected two cans
							return;
						moveforward();
						// increase y coordinate by one block distance
						ycoor = ycoor + 30.48;
					} else {
						correctPosition();
						turnRight(90);
					}
				}
				// move to next block and make a turn if reach the edge 
				moveforward();
				correctPosition();
				turnRight(90);
				xcoor = xcoor + 30.48;
			}

		}
		leftMotor.stop();
		rightMotor.stop();

	}


	/**
	 * Scans a tile by rotating the robot 90 degrees on a grid line intersection
	 * @param step
	 * Indicates how many blocks that robot passed 
	 * Determine which block to search for by identify the row and column
	 */
	public void scanTile(int step) {
		
		// correct the position upon arriving at each tile
		correctPosition();
		
		// if the step is even, can tile by rotating robot to the right
		if (step % 2 == 0) {
			// perform 10 rotations of 90 degrees to scan the current tile
			for (int i = 0; i < 10; i++) {
				turnRight(10);
				if(numcans == MAXNUMBERCANS) {
					turnTo(90);
					break;
				}
				// if the sampled distance is less than the threshold, stop rotation and retrieve the can
				if (fetchUS() < CAN_DIST) {
					leftMotor.stop();
					rightMotor.stop();
					getCan(i);
					
				}
			}
			
			correctPosition();
			turnLeft(90);
			odometer.setTheta(0);
			
			// if we have retrieved two cans, correct the track and return to starting position
			if (numcans == MAXNUMBERCANS) {
				correctPosition();
				correctTrack();
				RegularTravelTo(SZR_UR_x-SZR_LL_x,SZR_UR_y-SZR_LL_y, 0);
				stopMoving();
				return;
			}
		} 
		
		// when the step is odd, can tile by rotating to the left
		else {
			// perform 10 rotations of 90 degrees to scan the current tile
			for (int i = 0; i < 10; i++) {
				turnLeft(10);
				// if the sampled distance is less than the threshold, stop rotation and retrieve the can
				if(numcans == MAXNUMBERCANS) {
					turnTo(90);
					break;
				}
				if (fetchUS() < CAN_DIST) {
					turnLeft(5);
					leftMotor.stop();
					rightMotor.stop();
					getCan(i);
				}
			}
			//Correction after retrieving can
			correctPosition();
			turnRight(90);
			odometer.setTheta(180);//angle correction
			
			// if we have retrieved two cans, correct the track and return to starting position
			if (numcans == MAXNUMBERCANS) {
				correctPosition();
				correctTrack();
				RegularTravelTo(SZR_UR_x-SZR_LL_x,SZR_UR_y-SZR_LL_y, 1);
				stopMoving();
				return;
			}

		}
	}
	
	/**
	 * This method corrects the robot track position 
	 * Based on cases of number of heavy cans which have been retrieved
	 */
	public void correctTrack() {
		if (numheavy == 0) { //two light cans
			Project2.TRACK = 14.14;
			this.TRACK = 14.14;
		}
		if (numheavy == 1) { // one heavy can, one light can
			Project2.TRACK = 14.24;
			this.TRACK = 14.24;
		}
		if (numheavy == 2) { // two heavy cans 
			Project2.TRACK = 14.44;
			this.TRACK = 14.44;
		}
	}

	/**
	 * This method implement catch and identify can
	 * Upon detecting a can in the search region, approaches the can, performs color
	 * and weight identification then pulls the can into the storage area of the
	 * robot
	 */
	public void getCan(int i) {

		Sound.beep();
		clawMotor.setSpeed(CLAW_SPEED);
		
		// rotate the claw motor slightly, and drive towards the can
		clawMotor.rotate(convertAngle(-20), false);
		int distanceToTravel=20;
	
		//Reach and stay at appopriate distance from can
		RegularGoStraight(distanceToTravel);
		leftMotor.stop();
		rightMotor.stop();

		// close claw motor instance and get can weight
		WeightIdentification test = new WeightIdentification();
		clawMotor.close();
		int weight = test.getWeight();
		
		// if it is a heavy can, increase heavy can count
		if (weight == 1000) {
			numheavy++;
		}
		
		// drives backwards to seperate the claw from the can
		leftMotor.rotate(convertDistance(-5), true);
		rightMotor.rotate(convertDistance(-5), false);

		// re instantiate the claw motor as a regulated motor
		clawMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));


		// close the claw motor
		clawMotor.setSpeed(CLAW_SPEED);
		clawMotor.rotate(convertAngle(40), false);

		//drive away from the can
		leftMotor.rotate(convertDistance(-10), true);
		rightMotor.rotate(convertDistance(-10), false);

		// open the claw motor
		clawMotor.rotate(convertAngle(-60), false);

		// drive towards the can
		RegularGoStraight(distanceToTravel+5);

		// pull the can into the storage area, and open claw to make room for color sensor
		clawMotor.rotate(convertAngle(60), false);
		clawMotor.rotate(convertAngle(-60), false);

		// begin color identification with weighing result
		cc = new ColorCalibration(sensorMotor);
		cc.identifyColor(weight);

		// secure can in storage area and increase can number count
		clawMotor.rotate(convertAngle(60), false);
		numcans++;

		// return to searching path
		backtopath(distanceToTravel);
	}

	/**
	 *  Performs position correction by using the two light sensors.
	 *  When a light sensor detects a line it stops motion of the corresponding wheel
	 *  and waits for the second wheel to also align on the grid line, resulting in straight motion
	 */
	private void correctPosition() {

		//create boolean value stands for line detected status in left and right
		boolean rightLineDetected = false;
		boolean leftLineDetected = false;

		//slow down the speed for correction
		leftMotor.setSpeed(CORRECTION_SPEED);
		rightMotor.setSpeed(CORRECTION_SPEED);
		leftMotor.forward();
		rightMotor.forward();
		// Move the robot until one of the sensors detects a line
		while (!leftLineDetected && !rightLineDetected) {
			// when the right light sensor detects a grid line
			if (rightLS.fetch() < COLOR) { 
				rightLineDetected = true;
				// Stop the right motor
				this.stopMoving(false, true);

			} 
			// when the left light sensor detects a grid line
			else if (leftLS.fetch() < COLOR) {
				leftLineDetected = true;
				// Stop the left motor
				this.stopMoving(true, false);
			}
		}

		// Keep moving the left/right motor until both lines have been detected
		while ((!leftLineDetected || !rightLineDetected)) {
			// If the other line detected, stop the motors
			if (rightLineDetected && leftLS.fetch() < COLOR) {
				leftLineDetected = true;
				this.stopMoving();
			} else if (leftLineDetected && rightLS.fetch() < COLOR) {
				rightLineDetected = true;
				this.stopMoving();
			}
		}
		
		// reset motor speed and move off the grid line
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(-convertDistance(6.5), true);
		rightMotor.rotate(-convertDistance(6.5), false);

	}

	/**
	 * This method directs the robot back to the search path 
	 * after classifying and retrieving a can
	 * @param distance
	 */
	void backtopath(double distance) {
		leftMotor.rotate(-convertDistance(distance + 10), true);
		rightMotor.rotate(-convertDistance(distance + 10), false);
	}

	/**
	 * This method moves robot forward by 25cm
	 */
	void moveforward() {
		leftMotor.rotate(convertDistance(25), true);
		rightMotor.rotate(convertDistance(25), false);
	}

	/**
	 * Turns robot to a certain angle, always takes a minimum angle
	 * 
	 * @param theta
	 */
	void turnTo(double theta) {
		// determine the correction in angle required
		currentPosition = odometer.getXYT();
		double error = theta - currentPosition[2];
		if (error < -180.0) { // if the error is less than -180 deg
			error = error + 360; // add 360 degrees to the error, for a minimum angle
			turnRight(error);
		} else if (error > 180.0) { // if the error is greater than 180 deg
			error = error - 360; // subtract 360 degrees to the error, for a minimum angle
			turnLeft(error);
		} else if (error < 0) {
			turnLeft(error);
		} else {
			turnRight(error);
		}
	}
	
	public void turnUntil (double ang)
	{
		boolean isTurningRight = true;
//		boolean isTurningLeft = false;
		double angleToTurnTo = ang;
		
		if (ang>=360)
		{
			angleToTurnTo = ang-360;
		}
		
		while(isTurningRight)
		{
			if (Math.abs(odometer.getXYT()[2]-angleToTurnTo) < 2)
			{
				this.stopMoving();
				isTurningRight=false;
			}
			setSpeeds(200, -200);
		}		
		this.stopMoving();
	}

	/**
	 * Turns robot left by a certain angle
	 * @param theta (deg)
	 */
	void turnLeft(double theta) {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(-convertAngle(Math.abs(theta)), true);
		rightMotor.rotate(convertAngle(Math.abs(theta)), false);
	}

	/**
	 * Turns robot right by a certain angle
	 * @param theta (deg)
	 */
	void turnRight(double theta) {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertAngle(Math.abs(theta)), true);
		rightMotor.rotate(-convertAngle(Math.abs(theta)), false);
	}

	/**
	 * Moves robot to a certain point
	 * @param x (cm)
	 * @param y (cm)
	 */
	public void RegularTravelTo(double targetx, double targety) {
		isNavigating = true;
		double minAng = computeAngle(targetx, targety);
		this.turnTo(minAng);
		
		double deltaX = targetx*30.48 - odometer.getXYT()[0];
	    double deltaY = targety*30.48 - odometer.getXYT()[1];
	    double theta;
	    double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		
		if (deltaY == 0) {
	        if (deltaX >= 0) {
	          theta = 90;
	        } 
	        else {
	          theta = -90;
	        }
	      }
	    
	      //calculate theta that the robot should travel to 
	    else {
	    	theta = Math.atan2(deltaX, deltaY) * 180 / Math.PI; 
	    }

	    RegularGoStraight(distance); 
	    
	   //Correct current position
	    odometer.setX(targetx*30.48);
	    odometer.setY(targety*30.48);
	    odometer.setTheta(theta);
	    
	  isNavigating = false;
	  
	  try {
		Thread.sleep(350);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}	
	  }
	
	/**
	 * Moves robot to a certain point
	 * @param x (cm)
	 * @param y (cm)
	 */
	public void RegularTravelTo(double x, double y, int direction) {

		double deltaX = xcoor ;
		double deltaY = ycoor ;
		double theta;
		double deltaTheta;
		double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		if (deltaX == 0) {
			theta = 180;
		}

		// calculate the theta that the robot should travel to
		else {
			theta = Math.atan2(deltaX, deltaY) * 180 / Math.PI;
		}

		deltaTheta = theta;
		
		//ensure we make a minimum angle
		if (deltaTheta > 180) {
			deltaTheta -= 360;
		} else if (deltaTheta < -180) {
			deltaTheta += 360;
		}
		if (direction == 1) {
			turnRight(Math.abs(deltaTheta));
		}
		if (direction == 0) {
			turnLeft(Math.abs(deltaTheta));
		}
		
		RegularGoStraight(distance);
		this.turnUntil(anglebeforesearch);
		odometer.setTheta(anglebeforesearch);

	}

	/**
	 * This method compute angle for robot need to rotate for travel
	 * @param xvar (cm)
	 * @param yvar (cm)
	 */
	public double computeAngle(double targetx , double targety) {

		//get current position and compute variation in x and y coords
	double position[] = odometer.getXYT();
	double xvar = targetx*Project2.TILE_SIZE - position[0];
	double yvar = targety*Project2.TILE_SIZE - position[1];

		//calculate angle which robot needs to turn to in order to reach destination
	double ang = Math.atan2(xvar, yvar);
	double trajectoryAngle = ang*180/(3.14159)-position[2];
	
	if (trajectoryAngle < 0.5)
		trajectoryAngle += 360.0;
	
		//return
	return trajectoryAngle;
}
	
	/**
	 * This method moves robot straight by a certain distance
	 * 
	 * @param distance
	 */
	public void RegularGoStraight(double distance) {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(distance), true);
		rightMotor.rotate(convertDistance(distance), false);
	}

	/**
	 * This method make robot stop moving
	 * @param boolean stopLeft and stopRight control which motors will be stopped
	 */
	public void stopMoving(boolean stopLeft, boolean stopRight) {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		if (stopLeft)
			leftMotor.stop();
		if (stopRight)
			rightMotor.stop();
		leftMotor.endSynchronization();
	}

	public void stopMoving() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	
	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}
	
	/**
	 * Converts a distance in cm to the corresponding wheel rotations required
	 * 
	 * @param distance (cm)
	 * @return converted distance
	 */
	int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RADIUS));
	}

	/**
	 * Converts an angle in degrees to the corresponding wheel rotations required
	 * 
	 * @param direction (deg)
	 * @return converted angle
	 */
	int convertAngle(double angle) {
		return convertDistance(Math.PI * TRACK * angle / 360.0);
	}
	
	/**
	 * @return the sampled distance from the ultrasonic sensor
	 */
	private int fetchUS() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}

}