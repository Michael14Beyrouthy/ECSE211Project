package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.controller.LightSensorController;

/**
 * LightLocalizer class, performs Light Localization once US Localization is complete
 * @author Hongshuo Zhou
 *
 */
public class LightLocalizer {

	//Constants
	private final int FORWARD_SPEED;
	private final int ROTATE_SPEED;
	private final static double SENSOR_LENGTH = 6.5;
	public final double WHEEL_RAD = 2.09;
	public final double TRACK = 14.15;
	private double color = 0.30;

	//Instances of other classes
	private Odometer odometer;
	private static LightSensorController leftLS;
	private static LightSensorController rightLS;
	
	//Motors
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;

	/**
	 * Constructor for this class
	 * @param odometer odometer of the robot (singleton)
	 * @param leftLS left front light sensor that is used
	 * @param rightLS right front light sensor that is used
	 */
	public LightLocalizer(Odometer odometer, LightSensorController leftLS, LightSensorController rightLS, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor ) {
		this.odometer = odometer;
		this.FORWARD_SPEED = 250;
		this.ROTATE_SPEED = 150;		
		LightLocalizer.leftLS = leftLS;
		LightLocalizer.rightLS = rightLS;
		LightLocalizer.leftMotor = leftMotor;
		LightLocalizer.rightMotor = rightMotor;
	}

	/**
	 * Localizes the robot to the starting point using the two light sensors
	 * @param corner
	 */
	public void initialLocalize(long corner) {

		// Start moving the robot forward
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		this.moveForward();

		correct();

		this.travelDist(-SENSOR_LENGTH,175);
		this.turnBy(90,true);

		this.setSpeeds(150,150);
		this.moveForward();

		correct();

		this.travelDist(-SENSOR_LENGTH,175);

		this.turnBy(90, false); 

		//Sequence of three beeps 
		Sound.twoBeeps();
		Sound.beep();
		
		//Set position for robot determined by corresponding starting corner
		if (corner == 1)
		{
			odometer.setXYT(14*30.48, 30.48, 270.0);
		}
		if (corner == 2)
		{
			odometer.setXYT(14*30.48, 8*30.48, 180.0);
		}
		if (corner == 3)
		{
			odometer.setXYT(30.48, 8*30.48, 90.0);
		}
		if (corner == 0)
		{
			odometer.setXYT(30.48, 30.48, 0.0);
		}
		return;
	}
	
	/**
	 * This method moves robot forward 
	 */
	public void moveForward() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.endSynchronization();
	}

	/**
	 * This method makes the robot travel a certain distance with a certain speed
	 * @param distance
	 * @param speed
	 */
	public void travelDist(double distance, int speed) {

		resetMotors();
		setSpeeds(speed,speed);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
	}
	
	/**
	 * Resets the motors, setting their acceleration to a default 3000
	 */
	public void resetMotors() {
		// reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
	}
	
	/**
	 * Sets the speeds of the motors
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void setSpeeds(int leftSpeed, int rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
	}
	
	/**
	 * Converts a distance in cm to the corresponding wheel rotations required
	 * @param radius
	 * @param distance
	 * @return converted distance
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * Turns the robot by a certain angle
	 * @param dTheta
	 * @param clockwise
	 */
	public void turnBy(double dTheta, boolean clockwise) {
		if(clockwise) {
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), false);
		}
		else {
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), false);
		}
	}
	
	/**
	 * Converts an angle in degrees to the corresponding wheel rotations required
	 * @param radius
	 * @param width
	 * @param angle
	 * @return converted angle
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * Stops the robot from moving
	 * @param stopLeft
	 * @param stopRight
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
	
	/**
	 * Stops the robot from moving
	 */
	public void stopMoving() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	
	/**
	 * corrects the orientation of the robot with line detection
	 */
	private void correct() {

		boolean rightLineDetected = false;
		boolean leftLineDetected = false;
		double startingColor = rightLS.fetch();

		// Move the robot until one of the sensors detects a line
		while (!leftLineDetected && !rightLineDetected ) {
			if (Math.abs(rightLS.fetch()-startingColor) > 0.15) {
				rightLineDetected = true;
				// Stop the right motor
				this.stopMoving(false, true);

			} else if (Math.abs(leftLS.fetch()-startingColor) > 0.15) {
				leftLineDetected = true;

				// Stop the left motor
				this.stopMoving(true, false);
			}
		}

		// Keep moving the left/right motor until both lines have been detected
		while ((!leftLineDetected || !rightLineDetected)) {
			// If the other line detected, stop the motors
			if (rightLineDetected && Math.abs(leftLS.fetch()-startingColor) > 0.15) {
				leftLineDetected = true;
				this.stopMoving();
			} else if (leftLineDetected && Math.abs(rightLS.fetch()-startingColor) > 0.15) {
				rightLineDetected = true;
				this.stopMoving();
			}
		}

	}
}