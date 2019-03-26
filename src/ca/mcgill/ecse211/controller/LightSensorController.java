package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;

/**
 * This class implements the light sensor controller
 * used for line detection on the board
 * @author Hongshuo
 */
public class LightSensorController{
	
	private SensorModes lightSensor;
	private SensorMode idColour;
	private float[] colorValue;
	
	private float colorIntensity;
	//private TextLCD lcd;
	
	/**
	 * A constructor for this class
	 * @param mycolorleft light sensor to use
	 * 
	 */
	public LightSensorController(SensorModes mycolorleft) {
		this.lightSensor = mycolorleft;
		idColour = this.lightSensor.getMode(1);
		colorValue = new float[idColour.sampleSize()];
	}
	
	/**
	 * fetches samples from the light sensor 
	 * @return color intensity (float)
	 */
	public float fetch() {
		idColour.fetchSample(colorValue, 0);
		colorIntensity = colorValue[0];
		return colorIntensity;
	}	
}