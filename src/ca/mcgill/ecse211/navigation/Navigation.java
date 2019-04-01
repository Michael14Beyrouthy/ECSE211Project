package ca.mcgill.ecse211.navigation;
import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.project.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Navigation class, holds multiple methods to navigate the robot around the map accurately
 * @author Michael
 *
 */
public class Navigation extends Thread{
	
	
	public EV3LargeRegulatedMotor leftMotor;
	public EV3LargeRegulatedMotor rightMotor;
	Odometer odometer;	
	public boolean isNavigating = false;
	public boolean isNavigatingLX1 = false;
	public boolean isNavigatingLX2 = false;
	public boolean isNavigatingLY1 = false;	
	public boolean isNavigatingLY2 = false;
	public boolean isNavigatingLXY1 = false;	
	public boolean isNavigatingLXY2 = false;
	public boolean isNavigatingLYuXd1 = false;	
	public boolean isNavigatingLYuXd2 = false;
	public double sensorDist = 6.35; 
	private double color = 0.30;

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
		
		//odometer.setXYT(0.0, 0.0, 0.0);
		
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
	 * Moves robot towards target position.
	 * @param x
	 * @param y
	 */
	public void RegularTravelTo (double x, double y) {

	    isNavigating = true; 
	     
	    double deltaX = x - odometer.getXYT()[0];
	    double deltaY = y - odometer.getXYT()[1];
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

	    deltaTheta = theta - odometer.getXYT()[2];
	    
	    if (deltaTheta > 180) { 
	      deltaTheta -= 360;
	    }
	    else if (deltaTheta < -180) {
	      deltaTheta += 360;
	    }
	    
	    double minAng = computeAngle(x, y);
		//System.out.println("        " + minAng);	
			//turn to that angle
		this.turnTo(minAng);
	    
	    //turnTo(deltaTheta);

	    leftMotor.setSpeed(400);
	    rightMotor.setSpeed(400);
	    
	    RegularGoStraight(distance); 
	   
	    odometer.setX(x);
	    odometer.setY(y);
	    odometer.setTheta(theta);
	    
	  isNavigating = false;

	  }
	
/*=============================================================================================================
* 
* SET SPEED HERE (in the method below) (1/3)
*
*==============================================================================================================
*/
	
	/**
	 * Moves robot straight by a certain distance
	 * @param distance
	 */
	public void RegularGoStraight(double distance) {
		//leftMotor.setSpeed(100);
		//rightMotor.setSpeed(100);
		leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, distance), true);
	    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, distance), false);
	}
	
	/**
	 * Moves robot towards target position.
	 * @param targetx
	 * @param targety
	 */
	public void travelTo(double targetx, double targety) {

		isNavigating = true;
			//get angle
		double minAng = computeAngle(targetx, targety);
		System.out.println("        " + minAng);	
			//turn to that angle
		this.turnTo(minAng);
		//this.turnTo(0);
		
		while(isNavigating) {
			
				//consistently get current position and update distance to destination
			double position[] = odometer.getXYT();
			double dx = targetx * Project2.TILE_SIZE - position[0];
			double dy = targety * Project2.TILE_SIZE - position[1];
			double distance = Math.sqrt(dx*dx+ dy*dy);
			
				//set isNavigating to false once destination reached
			if (distance < 3) {
				isNavigating = false;
			}
			
				//isNavigating is true so make robot advance
			setSpeeds(Project2.FORWARD_SPEED, Project2.FORWARD_SPEED);
		}
		
		//isNavigating is false so stop robot 
		setSpeeds(0,0);
		
		}
	
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
		
		leftMotor.setSpeed(150);
	    rightMotor.setSpeed(150);
	    
	    RegularGoStraight(distance); 
	   
	    odometer.setX(targetx*30.48);
	    odometer.setY(targety*30.48);
	    odometer.setTheta(theta);
	    
	  isNavigating = false;
	  
	  }
		
		
	/**
	 * Moves robot towards target position.
	 * Goes in an L formation, correcting odometry and heading as it crosses each line. 
	 * Starts parallel to the Y axes, going up.
	 * 
	 * @param targetx
	 * @param targety
	 */
	public void TravelToLYup(double targetx, double targety) {
		
		isNavigatingLY1 = true;
		
		double minAng1 = computeAngle((odometer.getXYT()[0]/Project2.TILE_SIZE), targety);
		this.turnTo(minAng1);
		double startingXcoord = (int)(odometer.getXYT()[0]/30.48);
		double startingYcoord = (int)(odometer.getXYT()[1]/30.48);
		int counterY = 0;
		int counterX = 0;
		
		int counterY2 = 0;
		int counterX2 = 0;
		
		int YLinesToCross = (int) (targety - startingYcoord);
		int XLinesToCross = (int) (targetx - startingXcoord);

		double extraDistanceY = targety - YLinesToCross - startingYcoord;
		double extraDistanceX = targetx - XLinesToCross - startingXcoord;
		
		boolean left = false;
		boolean right = false;
		
		leftMotor.setSpeed(300);
		rightMotor.setSpeed(300);
		
		while(isNavigatingLY1) {
		
			while (counterY2<YLinesToCross)
			{
				this.moveForward();
				correct();
				
			Sound.beepSequenceUp();
			odometer.setY(((counterY+startingYcoord+1)*30.48)+sensorDist);
			odometer.setTheta(0.0);		
			counterY++;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
		    counterY2++;
			}
					
		    RegularGoStraight(extraDistanceY*Project2.TILE_SIZE-sensorDist); 
		    isNavigatingLY1 = false;
	}

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		this.turnTo(90);
		
		isNavigatingLY2 = true;
		
		leftMotor.setSpeed(300);
		rightMotor.setSpeed(300);
		
		while(isNavigatingLY2) {
			while (counterX2<XLinesToCross)
			{
				this.moveForward();
				correct();
				
			Sound.beepSequenceUp();
			odometer.setX(((counterX+startingXcoord+1)*30.48)+sensorDist);
			odometer.setTheta(90.0);		
			counterX++;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
		    counterX2++;
			}
					
		    RegularGoStraight(extraDistanceX*Project2.TILE_SIZE-sensorDist); 
		    
		    isNavigatingLY2 = false;
		}
		
	}
	
	/**
	 * Moves robot towards target position.
	 * Goes in an L formation, correcting odometry and heading as it crosses each line. 
	 * Starts parallel to the Y axes, going down.
	 * 
	 * @param targetx
	 * @param targety
	 */
	public void TravelToLYdown(double targetx, double targety) {
		
		isNavigatingLY1 = true;
		
		
		double minAng1 = computeAngle((odometer.getXYT()[0]/Project2.TILE_SIZE), targety);
		this.turnTo(minAng1);
		double startingXcoord = (int)(odometer.getXYT()[0]/30.48);
		double startingYcoord = (int)(odometer.getXYT()[1]/30.48);
		int counterY = 0;
		int counterX = 0;
		
		int counterY2 = 0;
		int counterX2 = 0;
		
		int YLinesToCross = Math.abs((int) (targety - startingYcoord));
		int XLinesToCross = Math.abs((int) (targetx - startingXcoord));
		
		double extraDistanceY = targety - YLinesToCross;
		double extraDistanceX = targetx - XLinesToCross;
		
		boolean left = false;
		boolean right = false;
		
/*=============================================================================================================
* 
* SET SPEED HERE (2/3)
*
*==============================================================================================================
*/
		
		leftMotor.setSpeed(325);
		rightMotor.setSpeed(325);
			
		while(isNavigatingLY1) {
		
			while (counterY2<YLinesToCross)
			{
				this.moveForward();
				correct();
				
			Sound.beepSequenceUp();
			odometer.setY(((startingYcoord-counterY)*30.48)-sensorDist);
			odometer.setTheta(180.0);		
			counterY++;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
		    counterY2++;
			}
					
		    RegularGoStraight(extraDistanceY*Project2.TILE_SIZE-sensorDist); 
		    
		    isNavigatingLY1 = false;
			
	}

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		this.turnTo(270);
		
		isNavigatingLY2 = true;
		
/*=============================================================================================================
* 
* SET SPEED HERE (3/3)
*
*==============================================================================================================
*/
		
		leftMotor.setSpeed(325);
		rightMotor.setSpeed(325);
		
		while(isNavigatingLY2) {
			while (counterX2<XLinesToCross)
			{
				this.moveForward();
				correct();
				
			Sound.beepSequenceUp();
			odometer.setX(((counterX+startingXcoord+1)*30.48)+sensorDist);
			odometer.setTheta(90.0);		
			counterX++;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
			//System.out.println("hi");
		    counterX2++;
			}
			
		    RegularGoStraight(extraDistanceX*Project2.TILE_SIZE-sensorDist); 
		    
		    isNavigatingLY2 = false;
		
		}
		
	}
	
public void TravelToLYupXdown(double targetx, double targety) {
		
		isNavigatingLYuXd1 = true;
		
		double minAng = computeAngle((odometer.getXYT()[0]/Project2.TILE_SIZE), targety);
		this.turnTo(minAng);
		double startingXcoord = (int)(odometer.getXYT()[0]/30.48);
		double startingYcoord = (int)(odometer.getXYT()[1]/30.48);
		int counterY = 0;
		int counterX = 0;
		
		int counterY2 = 0;
		int counterX2 = 0;
		
		int YLinesToCross = Math.abs((int) (targety - startingYcoord));
		int XLinesToCross = Math.abs((int) (targetx - startingXcoord))+1;

		double extraDistanceY = 0.5; //Math.abs(targety - YLinesToCross - startingYcoord);
		double extraDistanceX = 0.5; //Math.abs(targetx - XLinesToCross - startingXcoord);
		
		boolean left = false;
		boolean right = false;
		
		leftMotor.setSpeed(125);
		rightMotor.setSpeed(125);
		
		while(isNavigatingLYuXd1){
			while (counterY2<YLinesToCross)
			{
				this.moveForward();
				correct();
				
			Sound.beepSequenceUp();
			odometer.setY(((counterY+startingYcoord+1)*30.48)+sensorDist);
			odometer.setTheta(0.0);		
			counterY++;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
			//System.out.println("hi");
		    counterY2++;
			}
			
		    RegularGoStraight(extraDistanceY*Project2.TILE_SIZE-sensorDist); 
		    
		    isNavigatingLYuXd1 = false;
		
		} 

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		this.turnTo(270);
		
		isNavigatingLX2 = true;
		
/*=============================================================================================================
* 
* SET SPEED HERE (3/3)
*
*==============================================================================================================
*/
		
		leftMotor.setSpeed(125);
		rightMotor.setSpeed(125);
		
		while(isNavigatingLYuXd2) {
			
			while (counterX2<XLinesToCross)
			{
				this.moveForward();
				correct();
				
			Sound.beepSequenceUp();
			odometer.setX(((startingXcoord-counterX)*30.48)-sensorDist);
			odometer.setTheta(270.0);		
			counterX++;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
		    counterX2++;
			}
					
		    RegularGoStraight(extraDistanceX*Project2.TILE_SIZE-sensorDist); 
		    
		    isNavigatingLYuXd2 = false;
			
	}
		
	}

public void TravelToLXdown(double targetx, double targety) {
	
	isNavigatingLX1 = true;
	
	double minAng = computeAngle(targetx, (odometer.getXYT()[1]/Project2.TILE_SIZE));
	this.turnTo(minAng);
	double startingXcoord = (int)(odometer.getXYT()[0]/30.48);
	double startingYcoord = (int)(odometer.getXYT()[1]/30.48);
	int counterY = 0;
	int counterX = 0;
	
	int counterY2 = 0;
	int counterX2 = 0;
	
	int YLinesToCross = Math.abs((int) (targety - startingYcoord));
	int XLinesToCross = Math.abs((int) (targetx - startingXcoord))+1;

	double extraDistanceY = 0.5; //Math.abs(targety - YLinesToCross - startingYcoord);
	double extraDistanceX = 0.5; //Math.abs(targetx - XLinesToCross - startingXcoord);
	
	boolean left = false;
	boolean right = false;
	
	leftMotor.setSpeed(125);
	rightMotor.setSpeed(125);
	
	while(isNavigatingLX1) {
		
		while (counterX2<XLinesToCross)
		{
			this.moveForward();
			correct();
			
		Sound.beepSequenceUp();
		odometer.setX(((startingXcoord-counterX)*30.48)-sensorDist);
		odometer.setTheta(270.0);		
		counterX++;
		leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
	    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
	    counterX2++;
		}
				
	    RegularGoStraight(extraDistanceX*Project2.TILE_SIZE-sensorDist); 
	    
	    isNavigatingLX1 = false;
		
}

	try {
		Thread.sleep(1000);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	
	this.turnTo(90);
	
	isNavigatingLX2 = true;
	
/*=============================================================================================================
* 
* SET SPEED HERE (3/3)
*
*==============================================================================================================
*/
	
	leftMotor.setSpeed(125);
	rightMotor.setSpeed(125);
	
	while(isNavigatingLX2) {
		while (counterY2<YLinesToCross)
		{
			this.moveForward();
			correct();
			
		Sound.beepSequenceUp();
		odometer.setY(((counterY+startingYcoord+1)*30.48)+sensorDist);
		odometer.setTheta(0.0);		
		counterY++;
		leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
	    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
		//System.out.println("hi");
	    counterY2++;
		}
		
	    RegularGoStraight(extraDistanceY*Project2.TILE_SIZE-sensorDist); 
	    
	    isNavigatingLX2 = false;
	
	}
	
}

public void TravelToLXYdown(double targetx, double targety) {
	
	isNavigatingLXY1 = true;
	
	double minAng1 = computeAngle(targetx, (odometer.getXYT()[1]/Project2.TILE_SIZE));
	//this.turnTo(minAng1);
	double startingXcoord = (int)(odometer.getXYT()[0]/30.48);
	double startingYcoord = (int)(odometer.getXYT()[1]/30.48);
	int counterY = 0;
	int counterX = 0;
	
	int counterY2 = 0;
	int counterX2 = 0;
	
	int YLinesToCross = Math.abs((int) (targety - startingYcoord));
	int XLinesToCross = Math.abs((int) (targetx - startingXcoord));

	double extraDistanceY = Math.abs(targety - YLinesToCross - startingYcoord);
	double extraDistanceX = Math.abs(targetx - XLinesToCross - startingXcoord);
	
	boolean left = false;
	boolean right = false;
	
	leftMotor.setSpeed(125);
	rightMotor.setSpeed(125);
	
	while(isNavigatingLXY1) {
		
		while (counterX2<XLinesToCross)
		{
			this.moveForward();
			correct();
			
		Sound.beepSequenceUp();
		odometer.setX(((startingXcoord-counterX)*30.48)-sensorDist);
		odometer.setTheta(270.0);		
		counterX++;
		leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
	    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
	    counterX2++;
		}
				
	    RegularGoStraight(extraDistanceX*Project2.TILE_SIZE-sensorDist); 
	    
	    isNavigatingLXY1 = false;
		
}

	try {
		Thread.sleep(1000);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	
	this.turnTo(270);
	
	isNavigatingLXY2 = true;
	
/*=============================================================================================================
* 
* SET SPEED HERE (3/3)
*
*==============================================================================================================
*/
	
	leftMotor.setSpeed(125);
	rightMotor.setSpeed(125);
	
	while(isNavigatingLX2) {
		while (counterY2<YLinesToCross)
		{
			this.moveForward();
			correct();
			
		Sound.beepSequenceUp();
		odometer.setY(((startingYcoord-counterY)*30.48)-sensorDist);
		odometer.setTheta(180.0);		
		counterY++;
		leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
	    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
		//System.out.println("hi");
	    counterY2++;
		}
		
	    RegularGoStraight(extraDistanceY*Project2.TILE_SIZE-sensorDist); 
	    
	    isNavigatingLXY2 = false;
	
	}
	
}
	
/*public void TravelToLYdown(double targetx, double targety) {
		
		isNavigatingLY1 = true;
		leftMotor.setSpeed(65);
		rightMotor.setSpeed(65); 
		
		double minAng1 = computeAngle((odometer.getXYT()[0]/Project2.TILE_SIZE), targety);
		this.turnTo(minAng1);
		double startingXcoord = (int)(odometer.getXYT()[0]/30.48);
		double startingYcoord = (int)(odometer.getXYT()[1]/30.48);
		int counterY = 0;
		int counterX = 0;
		boolean left = false;
		boolean right = false;
		
		myColorStatusLeft.fetchSample(sampleColorLeft, 0);
		
		float tileColorL = sampleColorLeft[0];
		float tileColorR = sampleColorRight[0];
		
		leftMotor.forward();
		rightMotor.forward();
		
		while(isNavigatingLY1) {
		myColorStatusLeft.fetchSample(sampleColorLeft, 0);
		myColorStatusRight.fetchSample(sampleColorRight, 0);
		
		float diffR = sampleColorRight[0]-tileColorR;
		float diffL = sampleColorLeft[0]-tileColorL;
		
			//consistently get current position and update distance to destination
		double position[] = odometer.getXYT();
		double dx = 0;
		double dy = targety * Project2.TILE_SIZE - position[1];
		double distance = Math.sqrt(dx*dx+ dy*dy);
		//System.out.println("d1 = "+(int)distance);
		
		if (Math.abs(diffR)>0.1 && right == false)
		{
			Sound.beep();
			rightMotor.stop();
			right = true;
		}
		
		if (Math.abs(diffL)>0.1 && left == false)
		{
			Sound.beep();
			leftMotor.stop();
			left = true;
		}
		
		if (right == true && left == true)
		{
			Sound.buzz();
			odometer.setY(((startingYcoord-counterY)*30.48)-sensorDist);
			odometer.setTheta(0.0);		
			counterY++;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
			//System.out.println("hi");
			right = false;
			left = false;
			leftMotor.forward();
			rightMotor.forward();
		}
		
			//set isNavigating to false once destination reached
		if (distance < 1) {
			leftMotor.stop();
			rightMotor.stop();
			isNavigatingLY1 = false;
		}
	}

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		this.turnTo(90);
		
		isNavigatingLY2 = true;
		
		leftMotor.forward();
		rightMotor.forward();
		
		while(isNavigatingLY2) {
			myColorStatusLeft.fetchSample(sampleColorLeft, 0);
			myColorStatusRight.fetchSample(sampleColorRight, 0);
			
			float diffR = sampleColorRight[0]-tileColorR;
			float diffL = sampleColorLeft[0]-tileColorL;
			
				//consistently get current position and update distance to destination
			double position[] = odometer.getXYT();
			double dx = targetx * Project2.TILE_SIZE - position[0];
			double dy = 0;
			double distance = Math.sqrt(dx*dx+ dy*dy);
			
			if (Math.abs(diffR)>0.1 && right == false)
			{
				rightMotor.stop();
				right = true;
			}
			
			if (Math.abs(diffL)>0.1 && left == false)
			{
				leftMotor.stop();
				left = true;
			}
			
			if (right == true && left == true)
			{
				odometer.setX((counterX+startingXcoord+1)*30.48+sensorDist);
				odometer.setTheta(90.0);	
				counterX++;
				leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
			    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
				//System.out.println("hi2");
				right = false;
				left = false;
				leftMotor.forward();
				rightMotor.forward();
			}
			
				//set isNavigating to false once destination reached
			if (distance < 1) {
				leftMotor.stop();
				rightMotor.stop();
				isNavigatingLY2 = false;
			}
		}
		
	}*/

public void localizeBeforeTunnel(double xBeforeTunnel, double yBeforeTunnel)
{
	double minAng = computeAngle((odometer.getXYT()[0]/Project2.TILE_SIZE), yBeforeTunnel);
	//turn to that angle
	this.turnTo(minAng);	
	
	leftMotor.setSpeed(100);
	rightMotor.setSpeed(100);
	this.RegularGoStraight(Project2.TILE_SIZE/2);
	
	this.moveForward();
	correct();
	this.RegularGoStraight(-Project2.TILE_SIZE/2-sensorDist);
	
	this.turnTo(90);
	
	leftMotor.setSpeed(100);
	rightMotor.setSpeed(100);
	this.moveForward();
	correct();
	this.RegularGoStraight(Project2.TILE_SIZE/2-sensorDist);
}

public void localizeAfterTunnel(double xAfterTunnel, double yAfterTunnel)
{	
	leftMotor.setSpeed(100);
	rightMotor.setSpeed(100);
	this.moveForward();
	correct();
	this.RegularGoStraight(-Project2.TILE_SIZE/2-sensorDist);
	
	this.turnTo(90);
	
	leftMotor.setSpeed(100);
	rightMotor.setSpeed(100);
	this.moveForward();
	correct();
	this.RegularGoStraight(-Project2.TILE_SIZE/2-sensorDist);
}

	public void localizeBeforeTunnel2(double xBeforeTunnel, double yBeforeTunnel)
	{
		double minAng = computeAngle((odometer.getXYT()[0]/Project2.TILE_SIZE), yBeforeTunnel);
		//turn to that angle
		this.turnTo(minAng);	
		
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		this.RegularGoStraight(Project2.TILE_SIZE/2);
		
		this.moveForward();
		correct();
		this.RegularGoStraight(-Project2.TILE_SIZE/2-sensorDist);
		
		this.turnTo(270);
		
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		this.moveForward();
		correct();
		this.RegularGoStraight(Project2.TILE_SIZE/2-sensorDist);
	}
	
	public void localizeAfterTunnel2(double xAfterTunnel, double yAfterTunnel)
	{	
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		this.moveForward();
		correct();
		this.RegularGoStraight(-Project2.TILE_SIZE/2-sensorDist);
		
		this.turnTo(270);
		
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		this.moveForward();
		correct();
		this.RegularGoStraight(-Project2.TILE_SIZE/2-sensorDist);
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
	    	leftMotor.rotate(convertAngle(Project2.WHEEL_RAD, Project2.TRACK2, ang), true);
	    	rightMotor.rotate(-convertAngle(Project2.WHEEL_RAD, Project2.TRACK2, ang), false);
	    }
	    
	    
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
	
	
	/*public void traverseTunnel()
	{
		
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		boolean hot = true;
		boolean left = false;
		boolean right = false;

		
		leftMotor.setSpeed(125);
		rightMotor.setSpeed(125);
		
		myColorStatusLeft.fetchSample(sampleColorLeft, 0);
		
		float tileColor = sampleColorLeft[0];
		
		leftMotor.forward();
		rightMotor.forward();
		
		while(hot) {
		myColorStatusLeft.fetchSample(sampleColorLeft, 0);
		myColorStatusRight.fetchSample(sampleColorRight, 0);
		
		float diffR = sampleColorRight[0]-tileColor;
		float diffL = sampleColorLeft[0]-tileColor;
		
		if (Math.abs(diffR)>0.1 && right == false)
		{
			rightMotor.stop();
			right = true;
		}
		
		if (Math.abs(diffL)>0.1 && left == false)
		{
			leftMotor.stop();
			left = true;
		}
		
		if (right == true && left == true)
		{
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
			//System.out.println("hi");
			right = false;
			left = false;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 65), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 65), false);
		}
		
	}
	
	}
	*/
	

	/**
	 * Relocates robot before traversing the tunnel
	 */
	public void relocateBeforeTunnel2() {
		
		/*this.moveBackward();
		correct();
		RegularGoStraight(0.5*Project2.TILE_SIZE-sensorDist);*/
		this.turnTo(270);
		
		leftMotor.setSpeed(350);
		rightMotor.setSpeed(350);
		this.moveBackward();
		correct();
		RegularGoStraight(0.5*Project2.TILE_SIZE-sensorDist);
		this.turnTo(90);
	}
	
	/**
	 * Traverses the tunnel once robot is facing it
	 */
	public void traverseTunnel() {
		RegularGoStraight(3*Project2.TILE_SIZE);
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
	   * @return converted angle
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

			// Move the robot until one of the sensors detects a line
			while (!leftLineDetected && !rightLineDetected ) {
				if (rightLS.fetch() < color) {
					rightLineDetected = true;
					// Stop the right motor
					this.stopMoving(false, true);

				} else if (leftLS.fetch() < color) {
					leftLineDetected = true;

					// Stop the left motor
					this.stopMoving(true, false);
				}
			}

			// Keep moving the left/right motor until both lines have been detected
			while ((!leftLineDetected || !rightLineDetected)) {
				// If the other line detected, stop the motors
				if (rightLineDetected && leftLS.fetch() < color) {
					leftLineDetected = true;
					this.stopMoving();
				} else if (leftLineDetected && rightLS.fetch() < color) {
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
		public void stopMoving() {
			leftMotor.stop(true);
			rightMotor.stop(false);
		}
}
