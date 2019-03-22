package ca.mcgill.ecse211.navigation;


/**
 * Navigation.java
 * This class drives an EV3 robot to search points on the map; 
 */

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.project.Project2;
import ca.mcgill.ecse211.project.UltrasonicController;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;

public class Search implements UltrasonicController, NavigationController{
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor clawMotor=new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));;
	private EV3MediumRegulatedMotor sensorMotor=new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	// Current position of the robot [0] = X corr (cm), [1] = Y corr (cm), [2] = theta (deg)
	private static double currentPosition[] = new double[3];
	private Odometer odometer;
	private static boolean traveling; // Variable to track whether the robot is current navigating
	private int distance; // distance between the robot and the obstacle (cm)
	private SensorModes usSensor; // ultrasonic sensor
	// Current position of the robot [0] = X corr (cm), [1] = Y corr (cm), [2] =
	boolean isAvoiding = false; // variable to track when robot is avoiding an obstacle
	private double rDistance=0;
	private double rAngle=0;
	
	private EV3ColorSensor lcolorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private EV3ColorSensor rcolorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
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

	
	private static int SZR_UR_x =3;
	private static int SZR_UR_y =3;
	private static int SZR_LL_x =0;
	private static int SZR_LL_y =0;
	private int targetcolor = 0;
	private int step=0;
	
	private int total =0;
	private ColorCalibration cc;
	
	private boolean isNavigating = false;
	
	public Search( EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor,
			Odometer odometer, SensorModes usSensor, EV3ColorSensor left, EV3ColorSensor right) {
		// Reset the motors
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.setAcceleration(600);
		rightMotor.setAcceleration(600);
		this.usSensor = usSensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}


	public void run() {
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
	}
	public void searchcans(int SZR_LL[], int SZR_UR[], int targetcolor) {
		// Travel to each of the way-points
		SZR_UR_x=SZR_UR[0];
		SZR_UR_y=SZR_UR[1];
		
		SZR_LL_x=SZR_LL[0];
		SZR_LL_y=SZR_LL[1];
		
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

	}

	public void searching(int step) {
		System.out.println(step);
		if(step%2==0) {
		for(int i=0;i<10;i++) {
			turnRight(10);
		if(distance<30) {
			leftMotor.stop();
			rightMotor.stop();
			rDistance=distance;
			get();
		}
		}
		turnLeft(98);
		}
		else {
		for(int i=0;i<10;i++) {
			turnLeft(10);
		if(distance<30) {
			leftMotor.stop();
			rightMotor.stop();
			rDistance=distance;
			get();
		}
		}
		turnRight(98);
		}
	}
	
	
	public void get() {
		Sound.beep();
		leftMotor.rotate(convertDistance(distance)-10,true);
		rightMotor.rotate(convertDistance(distance)-10,false);	
		
		leftMotor.stop();
		rightMotor.stop();
		
		leftMotor.rotate(convertDistance(-10),true);
		rightMotor.rotate(convertDistance(-10),false);
		
		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(-50),false);
		
			
		leftMotor.rotate(convertDistance(25),true);
		rightMotor.rotate(convertDistance(25),false);
		int color=-1;
		cc= new ColorCalibration(sensorMotor);
		cc.identifyColor();
		if(cc.identifyColor()==targetcolor) {
			RegularTravelTo((double)SZR_UR_x,
		(double)SZR_UR_y);
		}
		
		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(50),false);		
		
		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(-50),false);		
		
		backtopath(rDistance);
		
		clawMotor.setSpeed(ROTATE_SPEED);
		clawMotor.rotate(convertAngle(50),false);		
		
	}
	public void lightcorrection() {
		long correctionStart, correctionEnd;

	    //get the first value of the tile 'brightness' to use a reference
	    lcolorSensor.getRedMode().fetchSample(lRGBValues,0);
	    lreferenceBrightness = lRGBValues[0];
	    
	    rcolorSensor.getRedMode().fetchSample(rRGBValues,0);
	    rreferenceBrightness = rRGBValues[0];
	    for(int i=0;i<4;i++) {
	    	leftstop=false;
	    	rightstop=false;
	    	leftMotor.setSpeed(70);
	    	rightMotor.setSpeed(70);
	    	leftMotor.forward();
	    	rightMotor.forward();
	    	
	    while (true) {
	      correctionStart = System.currentTimeMillis();

	      lcolorSensor.getRedMode().fetchSample(lRGBValues,0); //retrieve the current 'brightness' level
	      lbrightness = lRGBValues[0];
	      
	      rcolorSensor.getRedMode().fetchSample(rRGBValues,0); //retrieve the current 'brightness' level
	      rbrightness = rRGBValues[0];
	      
	      //If the current brightness differs from the reference brightness value by a value greater than the threshold
	      //the EV3 robot is traveling over a black line. 
	      if(Math.abs(lbrightness-lreferenceBrightness)*100 > diffThreshold) {
	    	leftMotor.stop();
	    	leftstop=true;
	      }
	      if(Math.abs(rbrightness-rreferenceBrightness)*100 > diffThreshold) {
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
	}

	void backtopath(double distance) {
		Sound.beep();
		Sound.beep();
		System.out.println("r  " +distance);
		leftMotor.rotate(-convertDistance(distance+7),true);
		rightMotor.rotate(-convertDistance(distance+7),false);
		
		
		
	}
	void movingforward() {
		leftMotor.rotate(convertDistance(30),true);
		rightMotor.rotate(convertDistance(30),false);
	}
	
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
	 * @param theta (deg)
	 */
	void turnLeft(double theta) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(Math.abs(theta)), true);
		rightMotor.rotate(convertAngle(Math.abs(theta)), false);
	}

	/**
	 * @param theta (deg)
	 */
	void turnRight(double theta) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(Math.abs(theta)), true);
		rightMotor.rotate(-convertAngle(Math.abs(theta)), false);
	}
	/**
	 * Converts a distance in centimeters to the corresponding wheel rotations required
	 * @param distance (cm)
	 * @return distance in wheel rotations 
	 */
	int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RADIUS));
	}
	/**
	 * Converts a direction in degrees to the corresponding wheel rotations required
	 * @param direction (deg)
	 * @return distance in wheel rotations 
	 */
	int convertAngle(double angle) {
		return convertDistance(Math.PI * TRACK * angle / 360.0);
	}





	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		filter(distance);
	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return this.distance;
	}
	public void filter(int distance) {
		int FILTER_OUT = 25;
		int filterControl = 0;
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance variable, however do increment the filter
			// value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing there: leave
			// the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave distance alone.
			filterControl = 0;
			this.distance = distance;
		}
	}
	
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
	    
	    turnTo(deltaTheta);

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
		leftMotor.rotate(convertDistance(distance), true);
	    rightMotor.rotate(convertDistance(distance), false);
	}
	
}
