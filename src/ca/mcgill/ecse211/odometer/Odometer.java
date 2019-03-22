/**
 * This class implements the odometer logic.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 * @author Jamie McLeish
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
 
  /**
   * Track width (distance between the center line of the two wheels, cm)
   */
	private static final double TRACK = 13.75; 	
  /**
   * Wheel radius (cm)
   */
	private static final double WHEEL_RAD = 2.09;
  /**
   * Odometer update period (ms)
   */
  private static final long ODOMETER_PERIOD = 25;
  
  // Position parameters 
  private double x;						// Robot x-axis coordinate (cm)
  private double y; 					// Robot y-axis coordinate (cm)
  private double theta;					// Robot position (deg)
  
  // Parameters used to determine robot position
  private int initLTachoCount;			// Initial left motor tachometer count
  private int initRTachoCount;			// Initial right motor tachometer count
  private int currLeftTachoCount;		// Current left motor tachometer count
  private int currRightTachoCount;		// Current right motor tachometer count
  
  /**
   * This is the default constructor of this class. It initiates all motors and variables once. It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
    
	odoData = OdometerData.getOdometerData(); // Allows access to x,y,z manipulation methods
    
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);
    this.initLTachoCount = 0;
    this.initRTachoCount = 0;
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
   * This method is meant to return the existing Odometer Object. It is meant to be used only if an
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
   * This method is where the logic for the odometer runs.
   */
  public void run() {
    long updateStart, updateEnd;
    
    // Variables used in the calculations of distance traveled
    double deltaDistL, deltaDistR;		// Distance traveled by left wheel and right wheel (rad)
    double deltaTheta;					// (deg)
    double dX=0, dY=0;					// Distance traveled in the X and Y directions (cm)
    double arcLengthR, arcLengthL;		// Total arc length travelled by each wheel (cm)
    double CenterArcLength;				// (cm)
    int tachoChangeL, tachoChangeR;		// Change in tahcometer count
    
    // Get the initial tachometer count
    initLTachoCount = leftMotor.getTachoCount();
    initRTachoCount = rightMotor.getTachoCount(); 
    
    while (true) {
       
      updateStart = System.currentTimeMillis();
     
      // Get the current tachometer count
      currLeftTachoCount = leftMotor.getTachoCount();
      currRightTachoCount = rightMotor.getTachoCount();

      // Calculate the change in tachometer count for each motor
      tachoChangeL = currLeftTachoCount - initLTachoCount;
      tachoChangeR = currRightTachoCount - initRTachoCount;
      
      // Update the tachometer count  
      initLTachoCount=currLeftTachoCount;
      initRTachoCount=currRightTachoCount;
      
      // Calculate change in radians traveled by each wheel
      deltaDistL = tachoChangeL*Math.PI/180.0;
      deltaDistR = tachoChangeR*Math.PI/180.0;
      
      // Calculate the total arc length traveled by each wheel
      arcLengthL = WHEEL_RAD*deltaDistL;		
      arcLengthR = WHEEL_RAD*deltaDistR;		

      //Calculate the center arc length
      CenterArcLength = (arcLengthR+arcLengthL)/2.0;
      deltaTheta=((arcLengthL-arcLengthR)/TRACK)*180/(Math.PI);
      theta=theta+deltaTheta;
       
      // Calculate the  change in the robot position
      dX = CenterArcLength * Math.sin(theta*Math.PI/180);
      dY = CenterArcLength * Math.cos(theta*Math.PI/180);
      
      // Update robot position
      x=x+dX;
      y=y+dY;
           
      // Update odometer
      odo.update(dX, dY, deltaTheta);

      // This ensures that the odometer only runs once every period
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
   * Method to manually set the x-axis position of the robot. Used for correction.
   * @param Xposition
   */
  public void setXCorr(double Xposition) {
	  x=Xposition;
  }
  /**
   * Method to manually set the y-axis position of the robot. Used for correction.
   * @param Yposition
   */
  public void setYCorr(double Yposition) {
	  y=Yposition;
  }
  public double getY() {
	  return getXYT()[1];
  }
  public double getX() {
	  return getXYT()[0];
  }
  public double getTheta() {
	  return getXYT()[2];
  }

  // method used to instantiate motors in future constructors
public EV3LargeRegulatedMotor[] getMotors() {
	return new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor };

}
}
