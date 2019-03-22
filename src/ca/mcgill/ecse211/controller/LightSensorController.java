package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;

/**
 * This class implements the light sensor controller
 * It is used in the navigation, odometryCorrection, lightLocalization classes
 * It is used for line detection on the board
 *
 */
public class LightSensorController{
	
	private SensorModes lightSensor;
	private SensorMode idColour;
	private float[] colorValue;
	
	private float colorIntensity;
	//private TextLCD lcd;
	
	/**
	 * This method is a constructor for this class
	 * @param mycolorleft light sensor to use
	 * @param lcd lcd screen on the ev3 block
	 */
	public LightSensorController(SensorModes mycolorleft) {
		this.lightSensor = mycolorleft;
		idColour = this.lightSensor.getMode(1);
		colorValue = new float[idColour.sampleSize()];
		//this.lcd = lcd;
	}
	
	/**
	 * This method fetches samples from the light sensor 
	 * @return color intensity (float)
	 */
	public float fetch() {
		idColour.fetchSample(colorValue, 0);
		colorIntensity = colorValue[0];
		return colorIntensity;
	}	
}