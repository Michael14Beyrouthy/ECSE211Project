package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.localization.*;
import ca.mcgill.ecse211.navigation.Navigation;
//import ca.mcgill.ecse211.weighing.*;


/**
 * Our project class, instantiates the motors and sets some constants
 * instantiates all threads and objects of other classes and runs them accordingly
 * @author micha
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
	
	/*private static final EV3ColorSensor leftLight = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final EV3ColorSensor rightLight = new EV3ColorSensor(LocalEV3.get().getPort("S4"));*/
	
	
	private static LightSensorController leftLS = new LightSensorController(myColorLeft);
	private static LightSensorController rightLS = new LightSensorController(myColorRight);
	

	//Robot related parameters
	public static final double WHEEL_RAD = 2.09;
	public static final double TRACK = 13.75;
	public static final double TILE_SIZE = 30.48;
	public static final int FORWARD_SPEED = 100, ROTATE_SPEED = 74;
	

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
		
		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor);
		Display odometryDisplay = new Display(lcd, odometer); 
		
		//Odometer odometer2 = Odometer.getOdometer(leftMotor, rightMotor);

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		// Instance  ultrasonicsensor 
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		// usDistance fetch samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");
		
		

		/*do {
			// clear display
			lcd.clear();

			// ask the user whether the motors should use rising or falling edge
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Rising |Falling ", 0, 2); 
			lcd.drawString(" Edge  |  Edge  ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		// Select which edge to use.
		if (buttonChoice == Button.ID_LEFT) {
			Risingorfalling = true;
		} else {
			Risingorfalling = false;
		}*/

		// Start odometer and display threads
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		//Thread odoThread2 = new Thread(odometer2);
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		//lcd.clear();
		
//		leftMotor.flt();
//		rightMotor.flt();
//		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
//		
//		odometer.setXYT(TILE_SIZE, TILE_SIZE, 0.0);
//
//		leftMotor.flt();
//		rightMotor.flt();
//		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
//		
//		odometer.setXYT(TILE_SIZE, TILE_SIZE, 0.0);
//
//		leftMotor.flt();
//		rightMotor.flt();
//		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		
		//System.out.println("hi");
		
		// Create ultrasonicsensor light localizer and navigation objects
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, false, usDistance);
		LightLocalizer lightLocalizer = new LightLocalizer(odometer, leftLS, rightLS, leftMotor, rightMotor);
		/*Navigation nav = new Navigation(odometer, myColorStatusRight, myColorStatusLeft);*/
		// start the ultrasonic localization
		//USLocalizer.localize();
			// run the light localization
		//lightLocalizer.initialLocalize();
		
		/*try {
			odoThread.wait();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}*/
		
		/*leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();*/
		
		/*Odometer odometer2 = Odometer.getOdometer(leftMotor, rightMotor);
		Thread odoThread2 = new Thread(odometer2);
		
		odoThread2.start();*/
		
		//nav.RegularTravelTo(2.5*TILE_SIZE, 3.5*TILE_SIZE);
				
		Navigation nav = new Navigation(odometer, leftLS, rightLS, leftMotor, rightMotor);
		
		//Search search = new Search
		
		/*nav.turnTo(45);
		nav.RegularGoStraight(TILE_SIZE*Math.sqrt(2)/2);*/
		
		nav.RegularTravelTo(.5*TILE_SIZE, .5*TILE_SIZE);
		
		nav.TravelToLYup((double)(WifiInfo.TNR_LL_x)+1-0.5, (double)(WifiInfo.TNR_LL_y)+0.5);
		
		//nav.RegularGoStraight(3*30.48);
		
		
		
		//nav.TravelToLYdown((double)(WifiInfo.SZR_LL_x), (double)(WifiInfo.SZR_LL_y));
		
		Sound.beep();
		Sound.beep();
		Sound.beep();
		Sound.beep();
		Sound.beep();
		
		nav.traverseTunnel();
		//nav.TravelToLYup((double)(WifiInfo.SZR_UR_x), (double)(WifiInfo.SZR_UR_y));
		
		
		
		
		//nav.RegularTravelTo(3.5, 5.5);
		
		//nav.traverseTunnel();
		
        //End process
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}