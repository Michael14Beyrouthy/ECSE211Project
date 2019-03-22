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

public class Navigation extends Thread{
	
	
	public EV3LargeRegulatedMotor leftMotor;
	public EV3LargeRegulatedMotor rightMotor;
	Odometer odometer;	
	public boolean isNavigating = false;
	public boolean isNavigatingLX1 = false;
	public boolean isNavigatingLX2 = false;
	public boolean isNavigatingLY1 = false;	
	public boolean isNavigatingLY2 = false;
	public double sensorDist = 6.5; 
	private double color = 0.30;

	private static LightSensorController leftLS;
	private static LightSensorController rightLS;
	
	/**
	 * Class constructor; takes in odometer and two sample providers
	 * 
	 * @param odometer
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
	 * computeAngle method; takes current position and direction, as well as target position
	 * and computes the angle at which our robot needs to turn to face the target position 
	 * 
	 * @param targetx
	 * @param targety
	 * @return
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
	 * travelTo method; takes in current and target position and moves robot towards target position.
	 * Uses computeAngle to determine where the robot needs to face
	 * Uses turnTo to turn to that angle
	 * 
	 * @param x
	 * @param y
	 */
	
	public void RegularTravelTo (double x, double y) {

	    isNavigating = true; 
	     
	    double deltaX = x - odometer.getX();
	    double deltaY = y - odometer.getY();
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

	    deltaTheta = theta - odometer.getTheta();
	    
	    if (deltaTheta > 180) { 
	      deltaTheta -= 360;
	    }
	    else if (deltaTheta < -180) {
	      deltaTheta += 360;
	    }
	    
	    double minAng = computeAngle(x, y);
		System.out.println("        " + minAng);	
			//turn to that angle
		this.turnTo(minAng);
	    
	    //turnTo(deltaTheta);

	    leftMotor.setSpeed(150);
	    rightMotor.setSpeed(150);
	    
	    RegularGoStraight(distance); 
	   
	    odometer.setX(x);
	    odometer.setY(y);
	    odometer.setTheta(theta);
	    
	  isNavigating = false;

	  }
	
	public void RegularGoStraight(double distance) {
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, distance), true);
	    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, distance), false);
	}
	
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
		
	/**
	 * travelToLYup method; takes in current and target position and moves robot towards target position.
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
		
		//System.out.println(YLinesToCross);
		//System.out.println(XLinesToCross);
		
		double extraDistanceY = targety - YLinesToCross;
		double extraDistanceX = targetx - XLinesToCross;
		
		boolean left = false;
		boolean right = false;
		
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		
		/*leftMotor.forward();
		rightMotor.forward();*/
		
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
			//System.out.println("hi");
		    counterY2++;
			}
		
		//consistently get current position and update distance to destination
			
		    RegularGoStraight(extraDistanceY*Project2.TILE_SIZE-sensorDist); 
		    
		    isNavigatingLY1 = false;
			
			/*double distance = targety * Project2.TILE_SIZE - odometer.getY();
		
			//set isNavigating to false once destination reached
		if (distance < 1) {
			leftMotor.stop();
			rightMotor.stop();
			isNavigatingLY1 = false;
		}*/
	}

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		this.turnTo(90);
		
		isNavigatingLY2 = true;
		
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		
		//leftMotor.forward();
		//rightMotor.forward();
		
		while(isNavigatingLY2) {
			while (counterX2<XLinesToCross)
			{
				this.moveForward();
				correct();
				
			Sound.beepSequenceUp();
			odometer.setX(((counterX+startingXcoord+1)*30.48)+sensorDist);
			odometer.setTheta(0.0);		
			counterX++;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
			//System.out.println("hi");
		    counterX2++;
			}
		
		//consistently get current position and update distance to destination
			
		    RegularGoStraight(extraDistanceX*Project2.TILE_SIZE-sensorDist); 
		    
		    isNavigatingLY2 = false;
			
			/*double distance = targetx * Project2.TILE_SIZE - odometer.getX();
		
			//set isNavigating to false once destination reached
		if (distance < 1) {
			leftMotor.stop();
			rightMotor.stop();
			isNavigatingLY1 = false;
		}*/
		
		}
		
	}
	
	/**
	 * travelToLYdown method; takes in current and target position and moves robot towards target position.
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
		
		//System.out.println(YLinesToCross);
		//System.out.println(XLinesToCross);
		
		double extraDistanceY = targety - YLinesToCross;
		double extraDistanceX = targetx - XLinesToCross;
		
		boolean left = false;
		boolean right = false;
		
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		
		/*leftMotor.forward();
		rightMotor.forward();*/
		
		while(isNavigatingLY1) {
		
			while (counterY2<YLinesToCross)
			{
				this.moveForward();
				correct();
				
			Sound.beepSequenceUp();
			odometer.setY(((startingYcoord-counterY)*30.48)-sensorDist);
			odometer.setTheta(0.0);		
			counterY++;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
			//System.out.println("hi");
		    counterY2++;
			}
		
		//consistently get current position and update distance to destination
			
		    RegularGoStraight(extraDistanceY*Project2.TILE_SIZE-sensorDist); 
		    
		    isNavigatingLY1 = false;
			
			/*double distance = targety * Project2.TILE_SIZE - odometer.getY();
		
			//set isNavigating to false once destination reached
		if (distance < 1) {
			leftMotor.stop();
			rightMotor.stop();
			isNavigatingLY1 = false;
		}*/
	}

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		this.turnTo(90);
		
		isNavigatingLY2 = true;
		
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		
		//leftMotor.forward();
		//rightMotor.forward();
		
		while(isNavigatingLY2) {
			while (counterX2<XLinesToCross)
			{
				this.moveForward();
				correct();
				
			Sound.beepSequenceUp();
			odometer.setX(((counterX+startingXcoord+1)*30.48)+sensorDist);
			odometer.setTheta(0.0);		
			counterX++;
			leftMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), true);
		    rightMotor.rotate(convertDistance(Project2.WHEEL_RAD, 1), false);
			//System.out.println("hi");
		    counterX2++;
			}
		
		//consistently get current position and update distance to destination
			
		    RegularGoStraight(extraDistanceX*Project2.TILE_SIZE-sensorDist); 
		    
		    isNavigatingLY2 = false;
			
			/*double distance = targetx * Project2.TILE_SIZE - odometer.getX();
		
			//set isNavigating to false once destination reached
		if (distance < 1) {
			leftMotor.stop();
			rightMotor.stop();
			isNavigatingLY1 = false;
		}*/
		
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
	

	/**
	 * turnTo method; takes in an angle and rotates the robot covering the shortest arc length to that angle.
	 * 
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
	 * setSpeeds method; takes in the desired speeds and sets the motor speeds accordingly
	 * 
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
	
	/*public int reading() {
		us.fetchSample(usData, 0); 
		return (int) (usData[0] * 100); 
	}*/ 
	
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
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	
	public void traverseTunnel() {
		
		this.moveBackward();
		correct();
		RegularGoStraight(0.5*Project2.TILE_SIZE-sensorDist);
		this.turnTo(270);
		
		this.moveBackward();
		correct();
		RegularGoStraight(0.5*Project2.TILE_SIZE-sensorDist);
		this.turnTo(90);
		
		RegularGoStraight(4*Project2.TILE_SIZE);
		
		
		
		
	}
	
	  public static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }

	  public static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }
	
	  public void moveForward() {
			leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
			leftMotor.startSynchronization();
			leftMotor.forward();
			rightMotor.forward();
			leftMotor.endSynchronization();
		}
	  
	  public void moveBackward() {
			leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
			leftMotor.startSynchronization();
			leftMotor.backward();
			rightMotor.backward();
			leftMotor.endSynchronization();
		}
	  
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
