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
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor clawMotor;
	private EV3MediumRegulatedMotor sensorMotor;
	// Current position of the robot [0] = X corr (cm), [1] = Y corr (cm), [2] = theta (deg)
	private static double currentPosition[] = new double[3];
	private Odometer odometer;
	private static boolean traveling; // Variable to track whether the robot is current navigating
	private int distance ; // distance between the robot and the obstacle (cm)
	private SensorModes usSensor; // ultrasonic sensor
	// Current position of the robot [0] = X corr (cm), [1] = Y corr (cm), [2] =
	boolean isAvoiding = false; // variable to track when robot is avoiding an obstacle
	private double rDistance=0;
	private double rAngle=0;
	private double color = 0.30;
	private int numcans = 0;
	  private float[] usData;
	  private SampleProvider usDistance;
	
	private float[] lRGBValues = new float[3];			//stores the sample retruned by the color sensor
	private float lreferenceBrightness;				//initial brightness level returned by the color sensor
	private float lbrightness;							//brightness level returned by the color sensor, used to identify black lines
	double diffThreshold=7; 	
	private boolean leftstop=false;
	private boolean rightstop=false;
	private float[] rRGBValues = new float[3];			//stores the sample retruned by the color sensor
	private float rreferenceBrightness;				//initial brightness level returned by the color sensor
	private float rbrightness;							//brightness level returned by the color sensor, used to identify black lines
	private static final long CORRECTION_PERIOD = 10; // odometer correction update period in ms

	private int SZR_UR_x=WifiInfo.SZR_UR_x;
	private int SZR_UR_y=WifiInfo.SZR_UR_y;
	private int SZR_LL_x=WifiInfo.SZR_LL_x;
	private int SZR_LL_y=WifiInfo.SZR_LL_y;
	private int targetcolor=3;
	private int step=0;
	private static LightSensorController leftLS;
	private static LightSensorController rightLS;
	
	private int total =0;
	private ColorCalibration cc;
	
	private boolean isNavigating = false;
	
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
	

/*	public void run() {
		try {
			Thread.sleep(4000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}
		// Travel to each of the way-points
		int column=SZR_UR_x-SZR_LL_x;
		int row=SZR_UR_y-SZR_LL_y;
		
		for(int i=0;i<4;i++){
			if(i%2!=0) {
			for(int j=row;j>-1;j--) {
			lightcorrection();
			System.out.println(i+"  "+j);
			rAngle=odometer.getTheta();
			if(j!=0) {
				searching(i);
				movingforward();
			}
			else
				turnLeft(90);
			}
			movingforward();
			turnLeft(90);
			}
			else {
				for(int j=0;j<row+1;j++) {
					System.out.println(i+"  "+j);
						lightcorrection();
						rAngle=odometer.getTheta();
						if(j!=row) {
						searching(i);
						movingforward();
						}
						else 
							turnRight(90);
				}
			movingforward();
			turnRight(90);
			}
			
			
		}
		leftMotor.stop();
		rightMotor.stop();
		
	}*/
	

	public void searchcans() {
		// Travel to each of the way-points
		
		int column=SZR_UR_x-SZR_LL_x;
		int row=SZR_UR_y-SZR_LL_y;


		for(int i=0;i<4;i++){
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
			}
			else {
				correct();
				turnLeft(90);
			}
			}
			movingforward();
			correct();
			turnLeft(90);
			}
			else {
				for(int j=0;j<row+1;j++) {
					
					
						rAngle=odometer.getTheta();
						if(j!=row) {
						searching(i);
						if(numcans==2)
							return;
						movingforward();
						}
						else {
							correct();
							turnRight(90);
						}
				}
			movingforward();
			correct();
			turnRight(90);
			}
			
			
		}
		leftMotor.stop();
		rightMotor.stop();

	}

	/**
	 * Searching process
	 * @param step
	 */
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
			if(numcans==2)
				return;
		}
		}
		correct();
		turnLeft(90);
		}
		else {
		for(int i=0;i<10;i++) {
			turnLeft(10);
		if(fetchUS()<25) {
			leftMotor.stop();
			rightMotor.stop();
			rDistance=fetchUS();
			get();
			if(numcans==2)
				return;
		}
		}
		correct();
		turnRight(90);
		
		}
	}
	
	/**
	 * Determines color of a can
	 */
	public void get() {
		System.out.println(distance);
		Sound.beep();
		leftMotor.rotate(convertDistance(fetchUS())+25,true);
		rightMotor.rotate(convertDistance(fetchUS())+25,false);	
		
		leftMotor.stop();
		rightMotor.stop();
		WeightIdentification test = new WeightIdentification(clawMotor);
		//test.getWeight();
		leftMotor.rotate(convertDistance(-10),true);
		rightMotor.rotate(convertDistance(-10),false);
		
		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(-50),false);
		
			
		leftMotor.rotate(convertDistance(25),true);
		rightMotor.rotate(convertDistance(25),false);
		
		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(50),false);
		
		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(-50),false);
		
		cc= new ColorCalibration(sensorMotor);
		cc.identifyColor();
		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(50),false);
	
		numcans++;
		backtopath(rDistance);
		if(numcans==2) {
			RegularTravelTo(SZR_LL_x,SZR_LL_y);
			turnTo(0);
			stopMoving();
			Sound.beepSequence();
			return;
		}
		
			
		/*if(cc.identifyColor()==targetcolor) {
			clawMotor.setSpeed(ROTATE_SPEED);
			clawMotor.rotate(convertAngle(50),false);		
			RegularTravelTo((double)SZR_UR_x,
		(double)SZR_UR_y);
		}*/
		
		
	}
	
	
	
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
	 * Corrects robot with light sensors
	/* 
	public void lightcorrection() {
		long correctionStart, correctionEnd;

	    //get the first value of the tile 'brightness' to use a reference
	    for(int i=0;i<4;i++) {
	    	leftstop=false;
	    	rightstop=false;
	    	leftMotor.setSpeed(70);
	    	rightMotor.setSpeed(70);
	    	leftMotor.forward();
	    	rightMotor.forward();
	    	
	    while (true) {
	      correctionStart = System.currentTimeMillis();
      
	      //If the current brightness differs from the reference brightness value by a value greater than the threshold
	      //the EV3 robot is traveling over a black line. 
	      if(leftLS.fetch() < color) {
	    	leftMotor.stop();
	    	leftstop=true;
	      }
	      if(rightLS.fetch() < color) {
	    	  rightMotor.stop();
	    	  rightstop=true;
	    	  
	      }
	      if(rightstop==true&&leftstop==true) {
	    	  leftMotor.rotate(-convertDistance(6.5),true);
	    	  rightMotor.rotate(-convertDistance(6.5),false);
	    	  turnLeft(90);
	    	  break;
	      }      
	      // this ensure the odometry correction occurs only once every period
	      correctionEnd = System.currentTimeMillis();
	      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
	        try {
	          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
	        } catch (InterruptedException e) {
	          // there is nothing to be done here
	        }
	      }
	    }
	    }
	}*/

	/**
	 * Puts the robot back on track
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
		System.out.println("currentPosition "+currentPosition[2]);
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
	public void RegularTravelTo (double x, double y) {

	    isNavigating = true; 
	     
	    double deltaX = x*30.48 - odometer.getXYT()[0];
	    double deltaY = y*30.48 - odometer.getXYT()[1];
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
	    
	    turnTo(deltaTheta);

	    leftMotor.setSpeed(250);
	    rightMotor.setSpeed(250);
	    
	    RegularGoStraight(distance); 
	   
	    odometer.setX(x);
	    odometer.setY(y);
	    odometer.setTheta(theta);
	    
	  isNavigating = false;

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



