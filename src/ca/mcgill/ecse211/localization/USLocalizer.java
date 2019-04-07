	package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.project.*;

/**
 * USLocalizer class, performs US localization to correct heading
 * @author Hongshuo
 *
 */
public class USLocalizer {

  // robot constants
  public static int ROTATION_SPEED = 200;
  private double deltaTheta;
  private static final double TURNING_ERROR = 3.5 ;
     
  private Odometer odometer;
  private float[] usData;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  private boolean Risingorfalling;
  private SampleProvider usDistance;

  //required ultrasonic constant
  private double d = 18.00;
  private double k = 2;

  /**
   * Class constructor
   * @param odo
   * @param leftMotor
   * @param rightMotor
   * @param localizationType
   * @param usDistance
   */
  public USLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			boolean localizationType, SampleProvider usDistance) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.Risingorfalling = localizationType;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
	}

	/**
	 * determines which localization to use
	 */
	public void localize() {
		if (Risingorfalling) {
			localizeRisingEdge();
		} else {
			localizeFallingEdge();
		}
	}

	/**
	 * Localize robot using rising edge
	 */
	public void localizeRisingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to the wall
		while (fetchUS() > d) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate until it sees the open space
		while (fetchUS() < d + k) {
			leftMotor.backward();
			rightMotor.forward();
		}
		Sound.beep();
		// record angle
		angleA = odometer.getXYT()[2];

		// rotate the other way until it sees the wall
		while (fetchUS() > d) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate until it sees open space
		while (fetchUS() < d + k) {
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.beep();
		angleB = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation, from tutorial slides
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2 + 180;
		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2 + 180;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		//rotate robot to theta = 0.0 using turning angle
		//introduce a fix error correction
		leftMotor.rotate(-convertAngle(Project2.WHEEL_RAD, Project2.TRACK, turningAngle-TURNING_ERROR-10), true);
		rightMotor.rotate(convertAngle(Project2.WHEEL_RAD, Project2.TRACK, turningAngle-TURNING_ERROR-10), false);

		// set theta = 0.0
		odometer.setTheta(0.0);
	}

	/**
	 * Localize robot using rising edge
	 */
	public void localizeFallingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to open space
		while (fetchUS() < d + k) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate to the first wall
		while (fetchUS() > d) {
			leftMotor.backward();
			rightMotor.forward();
		}
		Sound.buzz();
		// record angle
		angleA = odometer.getXYT()[2];

		// rotate out of the wall 
		while (fetchUS() < d + k) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate to the second wall
		while (fetchUS() > d) {
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.buzz();
		angleB = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 
        //introduce a fix error correction
		leftMotor.rotate(-convertAngle(Project2.WHEEL_RAD, Project2.TRACK, turningAngle-TURNING_ERROR-12), true);
		rightMotor.rotate(convertAngle(Project2.WHEEL_RAD, Project2.TRACK, turningAngle-TURNING_ERROR-12), false);

		// set odometer to theta = 0
		odometer.setTheta(0.0);

	}

	/**
	 * Gets the distance detected by the sensor
	 * @return distance read in cm
	 */
	private int fetchUS() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}

	/**
	 * Converts a distance in cm to the corresponding wheel rotations required
	 * @param radius
	 * @param distance
	 * @return converted distance
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Converts anm angle in degrees to the corresponding wheel rotations required
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return converted angle
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}