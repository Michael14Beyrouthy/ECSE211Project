package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.UltrasonicPoller;
import ca.mcgill.ecse211.localization.*;
import ca.mcgill.ecse211.navigation.*;

/**
 * Project class, instantiates the motors and sets some constants
 * Instantiates all threads and objects of other classes and runs them accordingly
 * @author Michael Beyrouthy
 * 
 */
public class Project2 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S2");
	
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
	public static double TRACK = 14; 
	public static final double TILE_SIZE = 30.48;
	public static final int FORWARD_SPEED = 100, ROTATE_SPEED = 150;
	
	//Navigation related parameters
	public static double startingX;
	public static double startingY;
	
	public static double Search_LL_x;
	public static double Search_LL_y;
	public static double Search_UR_x;
	public static double Search_UR_y;
	
	/**
	 * main() method of class 
	 * This is the method that runs when the whole project is run
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {
		
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

		// Create ultrasonicsensor light localizer and navigation objects
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, usDistance);
		LightLocalizer lightLocalizer = new LightLocalizer(odometer, leftLS, rightLS, leftMotor, rightMotor);
		
		WifiInfo wifi = new WifiInfo();
		wifi.getInfo();
		
		//Removes the wifi class messages so we can see the odometer readings
		System.out.println("");
		System.out.println("");
		System.out.println("");
		System.out.println("");
		System.out.println("");
		System.out.println("");
		System.out.println("");
		
		//Creates search object after getting the info from the wifi class
		//since it takes in the parameters instantly
		Search search = new Search(rightMotor, leftMotor,
				odometer,  usDistance,  leftLS,  rightLS, clawMotor, sensorMotor, TRACK);

		
		// start the ultrasonic localization
	    USLocalizer.localize();
	    // run the light localization
	    lightLocalizer.initialLocalize(WifiInfo.Corner);
		
	    //Depending on which corner we are in, set the starting X and Y to the points 
	    //we need to travel back to after getting the cans
	    if (WifiInfo.Corner == 0)
	    {
	    	startingX = 0.5;
			startingY = 0.5;
	    }
	    
	    if (WifiInfo.Corner == 1)
	    {
	    	startingX = 14.5;
			startingY = 0.5;
	    }
	    
	    if (WifiInfo.Corner == 2)
	    {
	    	startingX = 14.5;
			startingY = 8.5;
	    }
	    
	    if (WifiInfo.Corner == 3)
	    {
	    	startingX = 0.5;
			startingY = 8.5;
	    }
	    
	    if (WifiInfo.Search_LL_x == 0)
	    {
	    	Search_LL_x = WifiInfo.Search_LL_x+1;
	    }
	    else 
	    {
	    	Search_LL_x = WifiInfo.Search_LL_x;
	    }
	    
	    if (WifiInfo.Search_LL_y == 0)
	    {
	    	Search_LL_y = WifiInfo.Search_LL_y+1;
	    }
	    else 
	    {
	    	Search_LL_y = WifiInfo.Search_LL_y;
	    }
	    
	    //Navigation objects
		Navigation nav = new Navigation(odometer, leftLS, rightLS, leftMotor, rightMotor);
		
		/*
		 * The following is 4 possible conditions for our navigation, each being a combination 
		 * of one of two tunnel orientations and one of two tunnel coordinate being in our search zone 
		 */
		
		//Tunnel facing X direction
		if ((WifiInfo.Tunnel_UR_y-WifiInfo.Tunnel_LL_y) == 1 && (WifiInfo.Tunnel_UR_x-WifiInfo.Tunnel_LL_x) == 2)
		{
			//Tunnel LL is in our start zone	
			if (WifiInfo.Start_UR_x >= WifiInfo.Tunnel_LL_x && WifiInfo.Start_UR_y >= WifiInfo.Tunnel_LL_y && WifiInfo.Start_LL_x <= WifiInfo.Tunnel_LL_x && WifiInfo.Start_LL_y <= WifiInfo.Tunnel_LL_y)
			{
				nav.newTravelTo(WifiInfo.Tunnel_LL_x-0.5, WifiInfo.Tunnel_LL_y+0.5);
				nav.localizeBeforeTunnel((WifiInfo.Tunnel_LL_x-0.5)*30.48, (WifiInfo.Tunnel_LL_y+0.5)*30.48, 90);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((WifiInfo.Tunnel_UR_x+0.5)*30.48, (WifiInfo.Tunnel_UR_y-0.5)*30.48, 90);
				nav.newTravelTo(Search_LL_x, Search_LL_y);
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				nav.localizeBeforeSearchZone(Search_LL_x*30.48, Search_LL_y*30.48, 0);
				
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				search.searchcans();
				
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				nav.localizeAfterSearching(Search_LL_x*30.48, Search_LL_y*30.48);
				nav.newTravelTo(WifiInfo.Tunnel_UR_x+0.5, WifiInfo.Tunnel_UR_y-0.5);
				nav.localizeBeforeTunnel((WifiInfo.Tunnel_UR_x+0.5)*30.48, (WifiInfo.Tunnel_UR_y-0.5)*30.48, 270);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((WifiInfo.Tunnel_LL_x-0.5)*30.48, (WifiInfo.Tunnel_LL_y+0.5)*30.48, 270);
				nav.newTravelTo(startingX, startingY, 450);
				nav.dropCans();
				search.openClaw();
				
			}
			
			//Tunnel UR is in our start zone	
			else if (WifiInfo.Start_UR_x >= WifiInfo.Tunnel_UR_x && WifiInfo.Start_UR_y >= WifiInfo.Tunnel_UR_y && WifiInfo.Start_LL_x <= WifiInfo.Tunnel_UR_x && WifiInfo.Start_LL_y <= WifiInfo.Tunnel_UR_y)
			{
				nav.newTravelTo(WifiInfo.Tunnel_UR_x+0.5, WifiInfo.Tunnel_UR_y-0.5);
				nav.localizeBeforeTunnel((WifiInfo.Tunnel_UR_x+0.5)*30.48, (WifiInfo.Tunnel_UR_y-0.5)*30.48, 270);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((WifiInfo.Tunnel_LL_x-0.5)*30.48, (WifiInfo.Tunnel_LL_y+0.5)*30.48, 270);
				nav.newTravelTo(Search_LL_x, Search_LL_y);
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				nav.localizeBeforeSearchZone(Search_LL_x*30.48, Search_LL_y*30.48, 0);
				
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				
				search.searchcans();
				
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				nav.localizeAfterSearching(Search_LL_x*30.48, Search_LL_y*30.48);
				nav.newTravelTo(WifiInfo.Tunnel_LL_x-0.5, WifiInfo.Tunnel_LL_y+0.5);
				nav.localizeBeforeTunnel((WifiInfo.Tunnel_LL_x-0.5)*30.48, (WifiInfo.Tunnel_LL_y+0.5)*30.48, 90);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((WifiInfo.Tunnel_UR_x+0.5)*30.48, (WifiInfo.Tunnel_UR_y-0.5)*30.48, 90);
				nav.newTravelTo(startingX, startingY, 450);
			}
		}
		
		//Tunnel facing Y direction
		else if ((WifiInfo.Tunnel_UR_x-WifiInfo.Tunnel_LL_x) == 1 && (WifiInfo.Tunnel_UR_y-WifiInfo.Tunnel_LL_y) == 2)
		{
			//Tunnel LL is in our start zone	
			if (WifiInfo.Start_UR_x >= WifiInfo.Tunnel_LL_x && WifiInfo.Start_UR_y >= WifiInfo.Tunnel_LL_y && WifiInfo.Start_LL_x <= WifiInfo.Tunnel_LL_x && WifiInfo.Start_LL_y <= WifiInfo.Tunnel_LL_y)
			{
				nav.newTravelTo(WifiInfo.Tunnel_LL_x+0.5, WifiInfo.Tunnel_LL_y-0.5);
				nav.localizeBeforeTunnel((WifiInfo.Tunnel_LL_x+0.5)*30.48, (WifiInfo.Tunnel_LL_y-0.5)*30.48, 0);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((WifiInfo.Tunnel_UR_x-0.5)*30.48, (WifiInfo.Tunnel_UR_y+0.5)*30.48, 0);
				nav.newTravelTo(Search_LL_x, Search_LL_y);
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				nav.localizeBeforeSearchZone(Search_LL_x*30.48, Search_LL_y*30.48, 0);
				
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				
				search.searchcans();
				
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
												
				nav.localizeAfterSearching(Search_LL_x*30.48, Search_LL_y*30.48);
				nav.newTravelTo(WifiInfo.Tunnel_UR_x-0.5, WifiInfo.Tunnel_UR_y+0.5);
				nav.localizeBeforeTunnel((WifiInfo.Tunnel_UR_x-0.5)*30.48, (WifiInfo.Tunnel_UR_y+0.5)*30.48, 180);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((WifiInfo.Tunnel_LL_x+0.5)*30.48, (WifiInfo.Tunnel_LL_y-0.5)*30.48, 180);
				nav.newTravelTo(startingX, startingY, 450);
			}
			
			//Tunnel UR is in our start zone	
			else if (WifiInfo.Start_UR_x >= WifiInfo.Tunnel_UR_x && WifiInfo.Start_UR_y >= WifiInfo.Tunnel_UR_y && WifiInfo.Start_LL_x <= WifiInfo.Tunnel_UR_x && WifiInfo.Start_LL_y <= WifiInfo.Tunnel_UR_y)
			{
				nav.newTravelTo(WifiInfo.Tunnel_UR_x-0.5, WifiInfo.Tunnel_UR_y+0.5);
				nav.localizeBeforeTunnel((WifiInfo.Tunnel_UR_x-0.5)*30.48, (WifiInfo.Tunnel_UR_y+0.5)*30.48, 180);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((WifiInfo.Tunnel_LL_x+0.5)*30.48, (WifiInfo.Tunnel_LL_y-0.5)*30.48, 180);
				nav.newTravelTo(Search_LL_x, Search_LL_y);
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				nav.localizeBeforeSearchZone(Search_LL_x*30.48, Search_LL_y*30.48, 0);
				
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				search.searchcans();
				
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				nav.localizeAfterSearching(Search_LL_x*30.48, Search_LL_y*30.48);
				nav.newTravelTo(WifiInfo.Tunnel_LL_x+0.5, WifiInfo.Tunnel_LL_y-0.5);
				nav.localizeBeforeTunnel((WifiInfo.Tunnel_LL_x+0.5)*30.48, (WifiInfo.Tunnel_LL_y-0.5)*30.48, 0);
				nav.traverseTunnel();
				nav.localizeAfterTunnel((WifiInfo.Tunnel_UR_x-0.5)*30.48, (WifiInfo.Tunnel_UR_y+0.5)*30.48, 0);
				nav.newTravelTo(startingX, startingY, 450);
			}
		}
		
		//go straight to make sure we are in our starting corner
		nav.RegularGoStraight(8);
		
		//beep 5 times to indicate the end of our run
		//there is a little pause between each beep to make sure they are distinguishable
		Sound.beep();
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Sound.beep();
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Sound.beep();
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Sound.beep();
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Sound.beep();
		//odometer.setXYT(7*30.48, 1*30.48, 0);
		//search.searchcans();
		
		//End process
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
	
	/**
	 * Converts a distance in cm to the corresponding wheel rotations required
	 * @param distance
	 * @return
	 */
	static int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}

}