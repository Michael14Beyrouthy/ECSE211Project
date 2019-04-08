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
import ca.mcgill.ecse211.navigation.*;

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
	public static double TRACK = 14.1; // begin motion with default track (zero cans in storage area)
	//public static final double TRACK2= 14.1; //14.3 one heavy one light
	public static final double TILE_SIZE = 30.48;
	public static final int FORWARD_SPEED = 100, ROTATE_SPEED = 150;
	
	public static double startingX;
	public static double startingY;
	
	/**
	 * main() method of class 
	 * This is the method that runs when the whole project is run
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {

		WifiInfo wifi = new WifiInfo();
		wifi.getInfo();

		System.out.println("");
		System.out.println("");
		System.out.println("");
		System.out.println("");
		System.out.println("");
		System.out.println("");
		System.out.println("");
		
		
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
				odometer,  usDistance,  leftLS,  rightLS, clawMotor, sensorMotor, TRACK);

		// Create ultrasonicsensor light localizer and navigation objects
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, false, usDistance);
		LightLocalizer lightLocalizer = new LightLocalizer(odometer, leftLS, rightLS, leftMotor, rightMotor);
		// start the ultrasonic localization
	    USLocalizer.localize();
	    // run the light localization
	    lightLocalizer.initialLocalize(wifi.Corner);
	    
	    //startingX = odometer.getXYT()[0];
	    //startingY = odometer.getXYT()[1];
		
	    if (wifi.Corner == 0)
	    {
	    	startingX = 1;
			startingY = 1;
	    }
	    
	    if (wifi.Corner == 1)
	    {
	    	startingX = 14;
			startingY = 1;
	    }
	    
	    if (wifi.Corner == 2)
	    {
	    	startingX = 14;
			startingY = 8;
	    }
	    
	    if (wifi.Corner == 3)
	    {
	    	startingX = 1;
			startingY = 8;
	    }
	
		Navigation nav = new Navigation(odometer, leftLS, rightLS, leftMotor, rightMotor);
		
		//Tunnel facing X direction
		if ((wifi.Tunnel_UR_y-wifi.Tunnel_LL_y) == 1 && (wifi.Tunnel_UR_x-wifi.Tunnel_LL_x) == 2)
		{
			//Tunnel LL is in our start zone	
			if (wifi.Start_UR_x >= wifi.Tunnel_LL_x && wifi.Start_UR_y >= wifi.Tunnel_LL_y && wifi.Start_LL_x <= wifi.Tunnel_LL_x && wifi.Start_LL_y <= wifi.Tunnel_LL_y)
			{
				nav.newTravelTo(wifi.Tunnel_LL_x-0.5, wifi.Tunnel_LL_y+0.5);
				nav.localizeBeforeTunnel((wifi.Tunnel_LL_x-0.5)*30.48, (wifi.Tunnel_LL_y+0.5)*30.48, 90);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((wifi.Tunnel_UR_x+0.5)*30.48, (wifi.Tunnel_UR_y-0.5)*30.48, 90);
				nav.newTravelTo(wifi.Search_LL_x, wifi.Search_LL_y);
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				nav.localizeBeforeSearchZone(wifi.Search_LL_x*30.48, wifi.Search_LL_y*30.48, 0);
				
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				search.searchcans();
				
				try {
					Thread.sleep(3500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				nav.localizeAfterSearching(wifi.Search_LL_x*30.48, wifi.Search_LL_y*30.48);
				nav.newTravelTo(wifi.Tunnel_UR_x+0.5, wifi.Tunnel_UR_y-0.5);
				nav.localizeBeforeTunnel((wifi.Tunnel_UR_x+0.5)*30.48, (wifi.Tunnel_UR_y-0.5)*30.48, 270);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((wifi.Tunnel_LL_x-0.5)*30.48, (wifi.Tunnel_LL_y+0.5)*30.48, 270);
				nav.newTravelTo(startingX, startingY);
				
			}
			
			//Tunnel UR is in our start zone	
			else if (wifi.Start_UR_x >= wifi.Tunnel_UR_x && wifi.Start_UR_y >= wifi.Tunnel_UR_y && wifi.Start_LL_x <= wifi.Tunnel_UR_x && wifi.Start_LL_y <= wifi.Tunnel_UR_y)
			{
				nav.newTravelTo(wifi.Tunnel_UR_x+0.5, wifi.Tunnel_UR_y-0.5);
				nav.localizeBeforeTunnel((wifi.Tunnel_UR_x+0.5)*30.48, (wifi.Tunnel_UR_y-0.5)*30.48, 270);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((wifi.Tunnel_LL_x-0.5)*30.48, (wifi.Tunnel_LL_y+0.5)*30.48, 270);
				nav.newTravelTo(wifi.Search_LL_x, wifi.Search_LL_y);
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				nav.localizeBeforeSearchZone(wifi.Search_LL_x*30.48, wifi.Search_LL_y*30.48, 0);
				
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				search.searchcans();
				
				try {
					Thread.sleep(3500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				nav.newTravelTo(wifi.Tunnel_LL_x-0.5, wifi.Tunnel_LL_y+0.5);
				nav.localizeBeforeTunnel((wifi.Tunnel_LL_x-0.5)*30.48, (wifi.Tunnel_LL_y+0.5)*30.48, 90);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((wifi.Tunnel_UR_x+0.5)*30.48, (wifi.Tunnel_UR_y-0.5)*30.48, 90);
				nav.newTravelTo(startingX, startingY);
			}
		}
		
		//Tunnel facing Y direction
		else if ((wifi.Tunnel_UR_x-wifi.Tunnel_LL_x) == 1 && (wifi.Tunnel_UR_y-wifi.Tunnel_LL_y) == 2)
		{
			//Tunnel LL is in our start zone	
			if (wifi.Start_UR_x >= wifi.Tunnel_LL_x && wifi.Start_UR_y >= wifi.Tunnel_LL_y && wifi.Start_LL_x <= wifi.Tunnel_LL_x && wifi.Start_LL_y <= wifi.Tunnel_LL_y)
			{
				nav.newTravelTo(wifi.Tunnel_LL_x+0.5, wifi.Tunnel_LL_y-0.5);
				nav.localizeBeforeTunnel((wifi.Tunnel_LL_x+0.5)*30.48, (wifi.Tunnel_LL_y-0.5)*30.48, 0);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((wifi.Tunnel_UR_x-0.5)*30.48, (wifi.Tunnel_UR_y+0.5)*30.48, 0);
				nav.newTravelTo(wifi.Search_LL_x, wifi.Search_LL_y);
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				nav.localizeBeforeSearchZone(wifi.Search_LL_x*30.48, wifi.Search_LL_y*30.48, 0);
				
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				search.searchcans();
				
				try {
					Thread.sleep(3500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
												
				nav.localizeAfterSearching(wifi.Search_LL_x*30.48, wifi.Search_LL_y*30.48);
				nav.newTravelTo(wifi.Tunnel_UR_x-0.5, wifi.Tunnel_UR_y+0.5);
				nav.localizeBeforeTunnel((wifi.Tunnel_UR_x-0.5)*30.48, (wifi.Tunnel_UR_y+0.5)*30.48, 180);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((wifi.Tunnel_LL_x+0.5)*30.48, (wifi.Tunnel_LL_y-0.5)*30.48, 180);
				nav.newTravelTo(startingX, startingY);
			}
			
			//Tunnel UR is in our start zone	
			else if (wifi.Start_UR_x >= wifi.Tunnel_UR_x && wifi.Start_UR_y >= wifi.Tunnel_UR_y && wifi.Start_LL_x <= wifi.Tunnel_UR_x && wifi.Start_LL_y <= wifi.Tunnel_UR_y)
			{
				nav.newTravelTo(wifi.Tunnel_UR_x-0.5, wifi.Tunnel_UR_y+0.5);
				nav.localizeBeforeTunnel((wifi.Tunnel_UR_x-0.5)*30.48, (wifi.Tunnel_UR_y+0.5)*30.48, 180);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((wifi.Tunnel_LL_x+0.5)*30.48, (wifi.Tunnel_LL_y-0.5)*30.48, 180);
				nav.newTravelTo(wifi.Search_LL_x, wifi.Search_LL_y);
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				nav.localizeBeforeSearchZone(wifi.Search_LL_x*30.48, wifi.Search_LL_y*30.48, 0);
				
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				search.searchcans();
				
				try {
					Thread.sleep(3500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				nav.localizeAfterSearching(wifi.Search_LL_x*30.48, wifi.Search_LL_y*30.48);
				nav.newTravelTo(wifi.Tunnel_LL_x+0.5, wifi.Tunnel_LL_y-0.5);
				nav.localizeBeforeTunnel((wifi.Tunnel_LL_x+0.5)*30.48, (wifi.Tunnel_LL_y-0.5)*30.48, 0);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((wifi.Tunnel_UR_x-0.5)*30.48, (wifi.Tunnel_UR_y+0.5)*30.48, 0);
				nav.newTravelTo(startingX, startingY);
			}
		}
		//================================== Used for testing tunnel traversal =========================================================
		
		nav.newTravelTo(wifi.Tunnel_LL_x+0.5, wifi.Tunnel_LL_y-0.5);
		nav.localizeBeforeTunnel((wifi.Tunnel_LL_x+0.5)*30.48, (wifi.Tunnel_LL_y-0.5)*30.48, 0);
		nav.traverseTunnel();
		nav.localizeAfterTunnel((wifi.Tunnel_UR_x-0.5)*30.48, (wifi.Tunnel_UR_y+0.5)*30.48, 0);
		nav.newTravelTo(wifi.Search_LL_x, wifi.Search_LL_y);
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		nav.localizeBeforeSearchZone(wifi.Search_LL_x*30.48, wifi.Search_LL_y*30.48, 90);


		
//		nav.newTravelTo(3-0.5, 3+0.5);
//		nav.localizeBeforeTunnel((3-0.5)*30.48, (3+0.5)*30.48, 90);
//		nav.traverseTunnel();
//		nav.localizeAfterTunnel((5+0.5)*30.48, (4-0.5)*30.48, 90);
//		nav.newTravelTo(6, 6);
//		try {
//			Thread.sleep(2000);
//		} catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		nav.localizeBeforeSearchZone(6*30.48, 6*30.48);
		
		//===============================================End of area for testing tunnel traversal ====================================================================
        
		
		//===============================================Used for testing search =====================================================================================
		//call the search cans method, search start
		search.searchcans();
	  	
		//========================================End of testing for search ===========================================================================================
		
	    
//=========================================Start this code facing 0.0 at beginning of search zone===========================	    		
		
		//End process
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
	static int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}

}
