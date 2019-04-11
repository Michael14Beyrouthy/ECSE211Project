package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.project.Project2;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Odometer class, holds and constantly updates the values of the robot's odometer
 * @author Michael Beyrouthy
 */
public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  /**
   * @param leftTachoOld The previous left motor tacho data
   * @param rightTachoOld The previous right motor tacho data
   */
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private int leftTachoOld;
  private int rightTachoOld;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private static final long ODOMETER_PERIOD = 5; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   *
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {

    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z

    // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   *
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   *
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");
    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();
      double distL, distR, deltaD, dtheta;
      leftMotorTachoCount = leftMotor.getTachoCount(); // get tacho accounts
      rightMotorTachoCount = rightMotor.getTachoCount();

      // compute the distance traveled by the left wheel
      distL = Math.PI * Project2.WHEEL_RAD * (leftMotorTachoCount - leftTachoOld) / 180;

      // compute the distance traveled by the right wheel
      distR = Math.PI * Project2.WHEEL_RAD * (rightMotorTachoCount - rightTachoOld) / 180;

      leftTachoOld = leftMotorTachoCount; // save tacho counts for next iteration
      rightTachoOld = rightMotorTachoCount;

      deltaD = 0.5 * (distL + distR); // compute vehicle displacement at the center point

      dtheta = ((distL - distR) / ((Project2.TRACK))) * 180 / Math.PI; // compute change in heading.
      // It should be degree.

      double theta = (odo.getTheta()); // update heading
      theta = theta + dtheta;

      double dx = deltaD * Math.sin(Math.toRadians(theta));// compute the change of x component
      // of the displacement

      double dy = deltaD * Math.cos(Math.toRadians(theta));// compute the change of y component
      // of the displacement

      odo.update(dx, dy, dtheta); // updates the change of x, y, theta


      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

  /**
   * Motor accesor
   * 
   * @return EV3LargeRegulatedMotor[]
   */
	public EV3LargeRegulatedMotor[] getMotors() {
		return new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor };
	}


}
