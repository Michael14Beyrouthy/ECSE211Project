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
	
	
	// all robot motors, odometer
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor clawMotor;
	private EV3MediumRegulatedMotor sensorMotor;
	private Odometer odometer;
	private boolean isNavigating = false;
	
	private double anglebeforesearch=0;
	// distance between two front wheels 
	public double TRACK;
	
	// Current position of the robot [0] = X corr (cm), [1] = Y corr (cm), [2] = theta (deg)
	private static double currentPosition[] = new double[3];
	private SensorModes usSensor; // ultrasonic sensor
	
	private int numcans = 0; // keeps track of number of cans in storage area
	private int numheavy = 0; //keeps track of number of heavy cans
	
	private float[] usData;
	private SampleProvider usDistance;

	private double xcoor = 0;
	private double ycoor = 0;

	
	  private int search_SZR_UR_x=WifiInfo.Search_UR_x; 
	  private int search_SZR_UR_y=WifiInfo.Search_UR_y; 
	  private int search_SZR_LL_x=WifiInfo.Search_LL_x; 
	  private int search_SZR_LL_y=WifiInfo.Search_LL_y;
	 
	private static LightSensorController leftLS;
	private static LightSensorController rightLS;

	private ColorCalibration cc;

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
		if(search_SZR_UR_x == 15) {
			search_SZR_UR_x--;
		}
		if(search_SZR_UR_y == 9 ) {
			search_SZR_UR_y--;
		}
		// calculate search area size
		int column = search_SZR_UR_x - search_SZR_LL_x;
		int row = search_SZR_UR_y - search_SZR_LL_y;
		anglebeforesearch=odometer.getTheta();
		odometer.setTheta(0);
		// travel along the length of the search field, tile by tile
		for (int i = 0; i < column; i++) {
			
			if (numcans == 2) { // stop searching once we have collected two cans
				return;
			}
			
			//if the column count is odd
			if (i % 2 != 0) {
				for (int j = row; j > -1; j--) {
					if (j != 0) {
						scanTile(i);
						if (numcans == 2) { // stop searching once we have collected two cans
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
				moveforward();
				correctPosition();
				turnLeft(90);
				xcoor = xcoor + 30.48;
			} 
			
			//when column count is even
			else {
				for (int j = 0; j < row + 1; j++) {
					if (j != row) {
						scanTile(i);
						if (numcans == 2)
							return;
						moveforward();
						ycoor = ycoor + 30.48;
					} else {
						correctPosition();
						turnRight(90);
					}
				}
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
	 */
	public void scanTile(int step) {
		
		// correct the position upon arriving at each tile
		correctPosition();
		
		// if the step is even, can tile by rotating robot to the right
		if (step % 2 == 0) {
			// perform 10 rotations of 90 degrees to scan the current tile
			for (int i = 0; i < 10; i++) {
				turnRight(10);
				if(numcans == 2) {
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
			if (numcans == 2) {
				correctPosition();
				correctTrack();
				RegularTravelTo(search_SZR_UR_x-search_SZR_LL_x,search_SZR_UR_y-search_SZR_LL_y, 0);
				turnTo(0);
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
				if(numcans==2) {
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
			
			correctPosition();
			turnRight(90);
			odometer.setTheta(180);
			
			// if we have retrieved two cans, correct the track and return to starting position
			if (numcans == 2) {
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				correctPosition();
				correctTrack();
				RegularTravelTo(search_SZR_UR_x-search_SZR_LL_x,search_SZR_UR_y-search_SZR_LL_y, 1);
				turnTo(0);
				return;
			}

		}
	}
	
	/**
	 * Corrects the robot track position based on the number of heavy cans which have been retrieved
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
	 * Upon detecting a can in the search region, approaches the can, performs color
	 * and weight identification then pulls the can into the storage area of the
	 * robot
	 */
	public void getCan(int i) {

		Sound.beep();
		clawMotor.setSpeed(CLAW_SPEED);
		
		// rotate the claw motor slightly, and drive towards the can
		clawMotor.rotate(convertAngle(-20), false);
		int distanceToTravel;
		if(i<3 || i>6) {
			distanceToTravel = 21;
		}
		else{
			distanceToTravel = 31;
		}
		
		RegularGoStraight(distanceToTravel);
		leftMotor.stop();
		rightMotor.stop();

		// close claw motor instance and get can weight
		WeightIdentification test = new WeightIdentification();
		clawMotor.close();
		int weight = test.getWeight();
		
		// if it's a heavy can, increase heavy can count
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
		backtopath(distanceToTravel+5);
	}

	/**
	 *  Performs position correction by using the two light sensors.
	 *  When a light sensor detects a line it stops motion of the corresponding wheel
	 *  and waits for the second wheel to also align on the grid line, resulting in straight motion
	 */
	private void correctPosition() {

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
	 * Directs the robot back to the search path after classifying and retrieving a
	 * can
	 * 
	 * @param distance
	 */
	void backtopath(double distance) {
		leftMotor.rotate(-convertDistance(distance + 15), true);
		rightMotor.rotate(-convertDistance(distance + 15), false);
	}

	/**
	 * Moves robot forward by 25cm
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
	public void RegularTravelTo(double targetx, double targety/*, int direction*/) {
		isNavigating = true;
		double minAng = computeAngle(targetx, targety);
		this.turnTo(minAng);
		
		double deltaX = targetx*30.48 - odometer.getXYT()[0];
	    double deltaY = targety*30.48 - odometer.getXYT()[1];
	    double theta;
	    double deltaTheta;
	    double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		
		if (deltaY == 0) {
	        if (deltaX >= 0) {
	          theta = 90;
	        } 
	        else {
	          theta = -90;
	        }
	      }
	    
	      //calculate the theta that the robot should travel to 
	    else {
	    	theta = Math.atan2(deltaX, deltaY) * 180 / Math.PI; 
	    }
		
//		leftMotor.setSpeed(300);
//	    rightMotor.setSpeed(300);
	    
	    RegularGoStraight(distance); 
	   
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
		odometer.setTheta(anglebeforesearch);

	}

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
	 * Moves robot straight by a certain distance
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
	 * Make robot stop moving
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