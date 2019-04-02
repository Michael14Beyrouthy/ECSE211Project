package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.UltrasonicPoller;
import ca.mcgill.ecse211.localization.*;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.navigation.Search;
import ca.mcgill.ecse211.navigation.WeightIdentification;
//import ca.mcgill.ecse211.weighing.*;


/**
 * Project class, instantiates the motors and sets some constants
 * Instantiates all threads and objects of other classes and runs them accordingly
 * @author michael
 * 
 */
public class Project2 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S2");
	//private static boolean Risingorfalling = true;
	
	private static final Port portColourLeft = LocalEV3.get().getPort("S1");
	private static final SensorModes myColorLeft = new EV3ColorSensor(portColourLeft);
	private static final SampleProvider myColorStatusLeft = myColorLeft.getMode("RGB");
	
	private static final Port portColourRight = LocalEV3.get().getPort("S4");
	private static final SensorModes myColorRight = new EV3ColorSensor(portColourRight);
	private static final SampleProvider myColorStatusRight = myColorRight.getMode("RGB");
		
	private static final EV3LargeRegulatedMotor clawMotor=new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3MediumRegulatedMotor sensorMotor=new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	private static LightSensorController leftLS = new LightSensorController(myColorLeft);
	private static LightSensorController rightLS = new LightSensorController(myColorRight);
	private static UltrasonicController search;
	

	//Robot related parameters
	public static final double WHEEL_RAD = 2.09;
	public static final double TRACK = 14.4; //no cans
	//public static final double TRACK2= 14.13; //14.4 one heavy one light
	public static final double TILE_SIZE = 30.48;
	public static final int FORWARD_SPEED = 100, ROTATE_SPEED = 75;
	

	/**
	 * main() method of class 
	 * This is the method that runs when the whole project is run
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {

		WifiInfo wifi = new WifiInfo();
		wifi.getInfo();	

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor);
		Display odometryDisplay = new Display(lcd, odometer); 

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		// Instance  ultrasonicsensor 
		UltrasonicPoller usPoller = null;
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		
		// usDistance fetch samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];

		// Start odometer threads
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		//Start display thread
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();
		
		Search search = new Search(rightMotor, leftMotor,
				odometer,  usDistance,  leftLS,  rightLS, clawMotor, sensorMotor);

		// Create ultrasonicsensor light localizer and navigation objects
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, false, usDistance);
		LightLocalizer lightLocalizer = new LightLocalizer(odometer, leftLS, rightLS, leftMotor, rightMotor);
		// start the ultrasonic localization
	    USLocalizer.localize();
	    // run the light localization
		lightLocalizer.initialLocalize();
		
		try {
			odoThread.wait();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	
		Navigation nav = new Navigation(odometer, leftLS, rightLS, leftMotor, rightMotor);
		
		
		//================================== Used for testing tunnel traversal =========================================================
		
		
		//===============================================End of area for testing tunnel traversal ====================================================================
        
		
		//===============================================Used for testing search =====================================================================================
		//call the search cans method, search start
		search.searchcans();
	  	
		//========================================End of testing for search ===========================================================================================
		
	    
//=========================================Start this code facing 0.0 at beginning of search zone===========================	    

	    
/*	    odometer.setXYT(WifiInfo.SZR_LL_x, WifiInfo.SZR_LL_y, 0.0);
	    
	    nav.travelTo(WifiInfo.SZR_LL_x-0.5, WifiInfo.SZR_LL_y+0.5);
	    nav.TravelToLXdown(WifiInfo.TNR_UR_x+0.5, WifiInfo.SZR_UR_y-0.5);

	    nav.turnTo(270);
	    nav.traverseTunnel();
	    

	    nav.TravelToLXYdown(WifiInfo.Red_UR_x-0.5, WifiInfo.Red_UR_y-0.5);*/
	    
	    /*odometer.setXYT(6*TILE_SIZE, TILE_SIZE, 0.0);
	    
	    nav.newTravelTo(4, 2);
	    nav.localizeBeforeTunnel(3.5, 2.5);
	    nav.RegularGoStraight(TILE_SIZE*3);
	    nav.localizeAfterTunnel(0, 0);*/
	    
	    /*nav.travelTo(5.5, 1.5);
	    nav.TravelToLYupXdown(4.5, 4.5);

	    nav.turnTo(270);
	    nav.traverseTunnel();*/
		
		//odometer.setXYT(0, 0, 0.0);
	    
	    //baby nav.TravelToLXYdown(0.5, 0.5);
		
		//End process
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
	static int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}

}