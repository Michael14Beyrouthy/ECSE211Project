package ca.mcgill.ecse211.navigation;
import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.project.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;

/**
 * Navigation class, holds multiple methods to navigate the robot around the map accurately
 * @author Michael Beyrouthy
 *
 */
public class Navigation extends Thread{
	
	//motors
	public EV3LargeRegulatedMotor leftMotor;
	public EV3LargeRegulatedMotor rightMotor;
	
	//odometer
	Odometer odometer;
	
	//booleans and constants for methods
	public boolean isNavigating = false;
	public double sensorDist = 6.35; 
	private double color = 0.30;

	//instances of other classes
	private static LightSensorController leftLS;
	private static LightSensorController rightLS;
	
	/**
	 * Class constructor
	 * @param odometer
	 * @param leftLS
	 * @param rightLS
	 * @param leftMotor
	 * @param rightMotor
	 */
	public Navigation(Odometer odometer, LightSensorController leftLS, LightSensorController rightLS, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		
			//set odometer and sensor to input 
		this.odometer = odometer;
		this.leftLS = leftLS;
		this.rightLS = rightLS;
		
			//instantiate motors and sets accelerations
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;		
	    }
	
	/**
	 * Computes the angle by which to turn the robot for it to face a certain point from its current point
	 * @param targetx
	 * @param targety
	 * @return angle to turn the robot by
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
	 * Moves robot straight by a certain distance
	 * @param distance
	 */
	public void RegularGoStraight(double distance) {
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, distance), true);
	    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, distance), false);
	}
	
	public void RegularGoStraight(double distance, int speed) {
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
		leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, distance), true);
	    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, distance), false);
	}
	
	/**
	 * Moves robot towards target position.
	 * @param targetx
	 * @param targety
	 */
	public void newTravelTo(double targetx, double targety) {

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
	    
	    RegularGoStraight(distance, 275); 
	   
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
	 * Moves robot towards target position at a given speed
	 * @param targetx 
	 * @param targety
	 * @param speed
	 */
	public void newTravelTo(double targetx, double targety, int speed) {

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
	    
	    RegularGoStraight(distance, speed); 
	   
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
	 * Localizes robot before entering the tunnel
	 * @param xBeforeTunnel
	 * @param yBeforeTunnel
	 * @param facingTunnelAngle
	 */
	public void localizeBeforeTunnel(double xBeforeTunnel, double yBeforeTunnel, double facingTunnelAngle)
	{
		this.turnUntil(facingTunnelAngle);
		
		if ((xBeforeTunnel==0.5*30.48 && facingTunnelAngle==0) || (xBeforeTunnel==14.5*30.48 && facingTunnelAngle==180) || (yBeforeTunnel==0.5*30.48 && facingTunnelAngle==270) || (yBeforeTunnel==8.5*30.48 && facingTunnelAngle==0))
		{
			this.turnTo(270);
			this.RegularGoStraight(sensorDist-3);
			
			leftMotor.setSpeed(150);
			rightMotor.setSpeed(150);
			this.moveBackward();
			correct();
			this.RegularGoStraight(Project2.TILE_SIZE/2-sensorDist);

			this.turnTo(90);
		}
		
		else 
		{
			this.turnTo(90);
			this.RegularGoStraight(sensorDist-3);
		
			leftMotor.setSpeed(150);
			rightMotor.setSpeed(150);
			this.moveBackward();
			correct();
			this.RegularGoStraight(Project2.TILE_SIZE/2-sensorDist);
			
			this.turnTo(270);
		}
		
		odometer.setXYT(xBeforeTunnel, yBeforeTunnel, facingTunnelAngle);
	}
	
	/**
	 * Localizes the robot after exiting the tunnel
	 * @param xAfterTunnel
	 * @param yAfterTunnel
	 * @param leavingTunnelAngle
	 */
	public void localizeAfterTunnel(double xAfterTunnel, double yAfterTunnel, double leavingTunnelAngle)
	{
		this.RegularGoStraight(sensorDist, 400);
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		this.moveForward();
		correct();
		this.RegularGoStraight(-Project2.TILE_SIZE/2-sensorDist, 300);
		odometer.setXYT(xAfterTunnel, yAfterTunnel, leavingTunnelAngle);
		
	}
	
	/**
	 * Localizes the robot before beginning the search
	 * @param LLx
	 * @param LLy
	 * @param angle
	 */
	public void localizeBeforeSearchZone(double LLx, double LLy, double angle)
	{
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		this.turnUntil(angle+90);
		this.RegularGoStraight(-sensorDist, 300);
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		this.moveForward();
		correct();
		this.RegularGoStraight(-sensorDist, 150);
		this.turnTo(270);
		this.RegularGoStraight(-sensorDist, 300);
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		this.moveForward();
		correct();
		this.RegularGoStraight(-sensorDist, 150);
		odometer.setXYT(LLx, LLy, angle);
	}
	
	/**
	 * Localizes the robot after the search is over
	 * @param LLx
	 * @param LLy
	 */
	public void localizeAfterSearching(double LLx, double LLy)
	{
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		this.turnUntil(90);
		this.RegularGoStraight(-sensorDist, 300);
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		this.moveForward();
		correct();
		this.RegularGoStraight(-sensorDist, 150);
		this.turnTo(270);
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		this.moveForward();
		correct();
		this.RegularGoStraight(-sensorDist);
		odometer.setXYT(LLx, LLy, 0);
	}

	/**
	 * Turns robot by a certain angle
	 * @param ang
	 */
	public void turnTo(double ang)
	{
			//set rotate speeds
		leftMotor.setSpeed(Project2.ROTATE_SPEED);
		rightMotor.setSpeed(Project2.ROTATE_SPEED);
	    
			//if angle is bigger than 180, turn to angle following shortest arclength
	    if (ang>180)
	    {
	    	leftMotor.rotate(-convertAngle(Project2.WHEEL_RAD, Project2.TRACK, (360.0-ang)), true);
	    	rightMotor.rotate(convertAngle(Project2.WHEEL_RAD, Project2.TRACK, (360.0-ang)), false);
	    }
	    
	    	//if angle is less than 180, turn to that angle
	    else 
	    {
	    	leftMotor.rotate(convertAngle(Project2.WHEEL_RAD, Project2.TRACK, ang), true);
	    	rightMotor.rotate(-convertAngle(Project2.WHEEL_RAD, Project2.TRACK, ang), false);
	    }
	    
	    
	}
	
	/**
	 * Makes the robot turn until it is facing a certain angle
	 * @param ang
	 */
	public void turnUntil (double ang)
	{
		boolean isTurningRight = true;
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
	 * Sets motors' speeds and moves robot forward
	 * @param lSpd
	 * @param rSpd
	 */
	public void setSpeeds(float lSpd, float rSpd) {
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
	 * Sets motors' speeds and moves robot forward
	 * @param lSpd
	 * @param rSpd
	 */
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
	 * Traverses the tunnel once robot is facing it
	 */
	public void traverseTunnel() {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		RegularGoStraight(sensorDist+2, 175);
		this.moveForward();
		correct();
		RegularGoStraight(2.5*Project2.TILE_SIZE-sensorDist, 325);
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
	 * Converts an angle in degrees to the corresponding wheel rotations required
	 * @param radius
	 * @param width
	 * @param angle
	 *  @return converted angle
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	  /**
	   * Moves robot forward
	   */
	public void moveForward() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.endSynchronization();
	}
	  
	  /**
	   * Moves robot backward
	   */
	public void moveBackward() {
			leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
			leftMotor.startSynchronization();
			leftMotor.backward();
			rightMotor.backward();
			leftMotor.endSynchronization();
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
	  
	  /**
	   * Stops the robot
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
	   * Stops the robot
	   */
	  public void stopMoving() {
		  leftMotor.stop(true);
		  rightMotor.stop(false);
	  }

	
	  /**
	   * Drop the cans
	   */
	  public void dropCans() {
		  RegularGoStraight(sensorDist+3, 250);
	}
}
