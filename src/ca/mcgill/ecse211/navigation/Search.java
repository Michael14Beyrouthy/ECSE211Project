package ca.mcgill.ecse211.navigation;
import ca.mcgill.ecse211.controller.LightSensorController;
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

//change searchcans method 

/**
 * Search class, searches for cans in the search zone
 * @author Sumail
 *
 */
public class Search implements  NavigationController{
	public static double TRACK =14.3;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor clawMotor;
	private EV3MediumRegulatedMotor sensorMotor;
	// Current position of the robot [0] = X corr (cm), [1] = Y corr (cm), [2] = theta (deg)
	private static double currentPosition[] = new double[3];
	private Odometer odometer;
	private int distance ; // distance between the robot and the obstacle (cm)
	private SensorModes usSensor; // ultrasonic sensor
	boolean isAvoiding = false; // variable to track when robot is avoiding an obstacle
	private double rDistance=0;
	private double rAngle=0;
	private double color = 0.30;
	private int numcans = 0;
	private float[] usData;
	private SampleProvider usDistance;
	private int numheavy=0;
	
	private double xcoor=0;
	private double ycoor=0;					

	private int SZR_UR_x=WifiInfo.SZR_UR_x;
	private int SZR_UR_y=WifiInfo.SZR_UR_y;
	private int SZR_LL_x=WifiInfo.SZR_LL_x;
	private int SZR_LL_y=WifiInfo.SZR_LL_y;
	private static LightSensorController leftLS;
	private static LightSensorController rightLS;

	private ColorCalibration cc;

	public Search( EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor,
			Odometer odometer, SampleProvider usDistance,  LightSensorController leftLS, LightSensorController rightLS, EV3LargeRegulatedMotor clawMotor,EV3MediumRegulatedMotor sensorMotor 
	) {
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
		this.leftLS=leftLS;
		this.rightLS=rightLS;
		this.sensorMotor=sensorMotor;
		this.clawMotor=clawMotor;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];

	}

    /**
     * Main method for searching 
     * 
     * @param
     *
     */
	public void searchcans() {
		// Travel to each of the way-points
		
		//calculate search area size
		int column=SZR_UR_x-SZR_LL_x;
		int row=SZR_UR_y-SZR_LL_y;

		//travel to points throught the map
		for(int i=0;i<column;i++){
			if(numcans==2)
				return;
			if(i%2!=0) {
				for(int j=row;j>-1;j--) {
			
				rAngle=odometer.getTheta();
				if(j!=0) {
					searching(i);
					if(numcans==2)
						return;
					movingforward();
					ycoor=ycoor-30.48;
				}
				else {
					correct();
					turnLeft(90);
				}
			}
			movingforward();
			correct();
			turnLeft(90);
			xcoor=xcoor+30.48;
			}
			else {
				for(int j=0;j<row+1;j++) {
					
					rAngle=odometer.getTheta();
					if(j!=row) {
					searching(i);
					if(numcans==2)
						return;
					movingforward();
					ycoor=ycoor+30.48;
					}
					else {
						correct();
						turnRight(90);
					}
				}
			movingforward();
			correct();
			turnRight(90);
			xcoor=xcoor+30.48;
			}
			
			
		}
		leftMotor.stop();
		rightMotor.stop();

	}

	/**
	 * Searching process
	 * detects cans by rotating around point
	 * @param step
	 */
	public void searchinitial() {
		rDistance=0;
		rAngle=0;
		color = 0.30;
		numcans = 0;
		numheavy=0;
	    xcoor=0;
		ycoor=0;
		TRACK=14.3;
		odometer.setTheta(0);
	}
	public void searching(int step) {
		correct();
		if(step%2==0) {
		for(int i=0;i<10;i++) {
			turnRight(10);
		if(fetchUS()<25) {
			leftMotor.stop();
			rightMotor.stop();
			rDistance=fetchUS();
			get();	
		
		}
		}
		correct();
		turnLeft(90);
		odometer.setTheta(0);
		if(numcans==2) {
			correct();
			if(numheavy==0)
				TRACK=14.14;
			if(numheavy==1)
				TRACK=14.24;
			if(numheavy==2)
				TRACK=14.44;
			RegularTravelTo(SZR_LL_x,SZR_LL_y,0);
			turnTo(0);
			stopMoving();
			Sound.beepSequence();
			
			return;
		}
		}
		else {
		for(int i=0;i<10;i++) {
			turnLeft(10);
		if(fetchUS()<25) {
			turnLeft(5);
			leftMotor.stop();
			rightMotor.stop();
			rDistance=fetchUS();
			get();	
		}
		}
		correct();
		turnRight(90);
		odometer.setTheta(180);
		if(numcans==2) {
			correct();
			if(numheavy==0)
				TRACK=14.14;
			if(numheavy==1)
				TRACK=14.24;
			if(numheavy==2)
				TRACK=14.44;
			RegularTravelTo(SZR_LL_x,SZR_LL_y,1);
			turnTo(0);
			Sound.beepSequence();
			return;
		}
		
		}
	}
	
	/**
	 * Upon detecting a can in the search region, approaches the can, performs color and weight identification
	 * then pulls the can into the storage area of the robot
	 */
	public void get() {
		System.out.println(distance);
		Sound.beep();

		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(-20),false);
		//reach to the detected can 
		leftMotor.rotate(convertDistance(fetchUS()-15),true);
		rightMotor.rotate(convertDistance(fetchUS()-15),false);	
		
		leftMotor.stop();
		rightMotor.stop();
		
		//initialize weight new detection
		WeightIdentification test = new WeightIdentification();
		
		//close claw motor instance
		clawMotor.close();
		
		int weight= test.getWeight();
		leftMotor.rotate(convertDistance(-5),true);
		rightMotor.rotate(convertDistance(-5),false);
		
		//re instantiate the claw motor as a regulated motor
		clawMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
		
		if(weight==1000)
			numheavy++;
		
		
		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(40),false);
		

		leftMotor.rotate(convertDistance(-15),true);
		rightMotor.rotate(convertDistance(-15),false);
		
		clawMotor.rotate(convertAngle(-50),false);
		
			
		leftMotor.rotate(convertDistance(27),true);
		rightMotor.rotate(convertDistance(27),false);

		clawMotor.rotate(convertAngle(50),false);

		clawMotor.rotate(convertAngle(-50),false);
		
		//color identification with weighing result
		cc = new ColorCalibration(sensorMotor);
		cc.identifyColor(weight);
		
		//Pull can into storage area and increase can number count
		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(50),false);		
		numcans++;
		
		//return to searching path
		backtopath(rDistance);			
	}
	
    /**
     * method for correction
     * Ensure position accuracy
     *
     */
	private void correct() {

		boolean rightLineDetected = false;
		boolean leftLineDetected = false;
		
		leftMotor.setSpeed(150);
    	rightMotor.setSpeed(150);
    	leftMotor.forward();
    	rightMotor.forward();
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
		leftMotor.setSpeed(250);
    	rightMotor.setSpeed(250);
		leftMotor.rotate(-convertDistance(6.5),true);
  	  	rightMotor.rotate(-convertDistance(6.5),false);
  	 
		

	}

	/**
	 * Directs the robot back to the search path after classifying and retrieving a can
	 * @param distance
	 */
	void backtopath(double distance) {
		System.out.println("r  " +distance);
		leftMotor.rotate(-convertDistance(distance+13),true);
		rightMotor.rotate(-convertDistance(distance+13),false);
		}
	
	/**
	 * Moves robot forward a certain distance
	 */
	void movingforward() {
		leftMotor.rotate(convertDistance(25),true);
		rightMotor.rotate(convertDistance(25),false);
	}
	
	/**
	 * Turns robot to a certain angle
	 * @param theta
	 */
	void turnTo(double theta) {
		// determine the correction in angle required
		currentPosition = odometer.getXYT();
		double error = theta - currentPosition[2];
		System.out.println("theta  "+theta );
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
	 * @param theta
	 */
	void turnLeft(double theta) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(Math.abs(theta)), true);
		rightMotor.rotate(convertAngle(Math.abs(theta)), false);
	}

	/**
	 * Turns robot right by a certain angle
	 * @param theta
	 */
	void turnRight(double theta) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(Math.abs(theta)), true);
		rightMotor.rotate(-convertAngle(Math.abs(theta)), false);
	}
	
	/**
	 * Converts a distance in cm to the corresponding wheel rotations required
	 * @param distance (cm)
	 * @return converted distance
	 */
	int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RADIUS));
	}
	
	/**
	 * Converts an angle in degrees to the corresponding wheel rotations required
	 * @param direction (deg)
	 * @return converted angle
	 */
	int convertAngle(double angle) {
		return convertDistance(Math.PI * TRACK * angle / 360.0);
	}

	
	private int fetchUS() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}
	/**
	 * Processes the distance read by the US sensor
	 */

	
	/**
	 * Moves robot to a certain point
	 * @param x
	 * @param y
	 */
	public void RegularTravelTo (double x, double y, int direction) {
		 
	    double deltaX = xcoor - x*30.48;
	    double deltaY = ycoor - y*30.48;
	    double theta;
	    double deltaTheta;
	    double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
	    if (deltaX == 0) {
	       theta=180;
	      }
	    
	      //calculate the theta that the robot should travel to 
	    else {
	    	theta = Math.atan2(deltaX, deltaY) * 180 / Math.PI; 
	    }    
	    
	    deltaTheta = theta;
	    System.out.println("theta  "+theta);
	    System.out.println("odometer theta "+odometer.getXYT()[2]);
	    if (deltaTheta > 180) { 
	      deltaTheta -= 360;
	    }
	    else if (deltaTheta < -180) {
	      deltaTheta += 360;
	    }
	    if(direction==1)
	    turnRight(Math.abs(deltaTheta));
	    if(direction==0)
		 turnLeft(Math.abs(deltaTheta));

	    leftMotor.setSpeed(250);
	    rightMotor.setSpeed(250);
	    
	    RegularGoStraight(distance); 
	   
	    odometer.setX(0);
	    odometer.setY(0);
	    
	  
	  }

	/**
	 * Moves robot straight by a certain distance
	 * @param distance
	 */
	public void RegularGoStraight(double distance) {
		leftMotor.setSpeed(250);
		rightMotor.setSpeed(250);
		leftMotor.rotate(convertDistance(distance), true);
	    rightMotor.rotate(convertDistance(distance), false);
	}
	
	
	
	/** Make robot stop moving
	 * @param 
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



