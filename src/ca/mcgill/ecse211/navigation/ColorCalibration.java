package ca.mcgill.ecse211.navigation;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * ColorCalibration class, determines color of a can
 * @author Jamie
 *
 */
public class ColorCalibration {
	
	
	private static double[] redArray = new double [10];
	private static double[] greenArray = new double [10];
	private static double[] blueArray = new double [10];
	
	private static double redMean, greenMean, blueMean;
	
	/*
	 * Target RBG Mean values for a GREEN can
	 */
	final private static double GREEN_TR = 0.302959415;
	final private static double GREEN_TB = 0.263884275;
	final private static double GREEN_TG = 0.947320124;
	
	/*
	 * Target RBG Mean values for a BLUE can
	 */
	final private static double BLUE_TR = 0.384866034;
	final private static double BLUE_TB = 0.709316646;
	final private static double BLUE_TG = 0.645135536;
	
	/*
	 * Target RBG Mean values for a RED can
	 */
	final private static double	RED_TR = 0.840905986;
	final private static double RED_TB = 0.229910294;
	final private static double RED_TG = 0.320875403;
	
	/*
	 * Target RBG Mean values for a YELLOW can
	 */
	final private static double YELLOW_TR = 0.66308114;
	final private static double YELLOW_TB = 0.298730941;
	final private static double YELLOW_TG = 0.518506479;

	
	private EV3MediumRegulatedMotor sensorMotor;
	
	private boolean onLeftSide;
	
	
	private static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	private static float[] RGBValues = new float[3]; // Stores the sample returned by the color sensor

	/**
	 * Class constructor
	 * @param sensorMotor
	 */
	public ColorCalibration(EV3MediumRegulatedMotor sensorMotor) {
		this.sensorMotor = sensorMotor;
		onLeftSide = false;
		sensorMotor.setSpeed(120);
	}
	
	
	/**
	 * Identifies the color of the can in front of the robot
	 * @return 1, 2, 3 or 4 depending on which colour was detected
	 */
	public int identifyColor(int weight) {
		
		//rotate the sensor 90 degrees, take a sample of the can color each 9 degrees
		int i = 0;
		if (!onLeftSide) {
			while (i < 10) {
				// rotate the sensor around the can, take a sample of the can color each 9 degrees
				sensorMotor.rotate(-14, false);// rotate the sensor motor 9 degrees
				colorSensor.getRGBMode().fetchSample(RGBValues, 0); // acquire data from color sensor
				// Store the RGB Values in an array
				redArray[i] = RGBValues[0];
				greenArray[i] = RGBValues[1];
				blueArray[i] = RGBValues[2];
				i++;
			}
			sensorMotor.rotate(160,false);
			onLeftSide = true;
		}
		else {
			while (i < 10) {
				// rotate the sensor around the can, take a sample of the can color each 9 degrees
				sensorMotor.rotate(11, false);// rotate the sensor motor 9 degrees
				colorSensor.getRGBMode().fetchSample(RGBValues, 0); // acquire data from color sensor
				// Store the RGB Values in an array
				redArray[i] = RGBValues[0];
				greenArray[i] = RGBValues[1];
				blueArray[i] = RGBValues[2];
				i++;
			}
			onLeftSide = false;
		}
		computeMean();
		
		//compute the euclidean error for each can color (yellow, blue, green, red)
		double eGreen = Math.sqrt(Math.pow((redMean-GREEN_TR),2)+Math.pow((blueMean-GREEN_TB),2)+Math.pow((greenMean-GREEN_TG),2));
		double eYellow = Math.sqrt(Math.pow((redMean-YELLOW_TR),2)+Math.pow((blueMean-YELLOW_TB),2)+Math.pow((greenMean-YELLOW_TG),2));
		double eBlue = Math.sqrt(Math.pow((redMean-BLUE_TR),2)+Math.pow((blueMean-BLUE_TB),2)+Math.pow((greenMean-BLUE_TG),2));
		double eRed = Math.sqrt(Math.pow((redMean-RED_TR),2)+Math.pow((blueMean-RED_TB),2)+Math.pow((greenMean-RED_TG),2));

		//identify can color ID based on smallest eclidean error mean
		double sortColors[] = {eGreen, eYellow, eBlue, eRed};
		Arrays.sort(sortColors);
	
		if(sortColors[0]== eGreen) {
			System.out.println("GREEN");
			Sound.playTone(1000, weight);
			
			Sound.playTone(1000, weight);
			return 2;
		}
		if(sortColors[0]==eRed) {
			System.out.println("RED");
			Sound.playTone(1000, weight);
			Sound.playTone(1000, weight);
			Sound.playTone(1000, weight);
			Sound.playTone(1000, weight);
			return 4;
		}
		if(sortColors[0]==eYellow) {
			System.out.println("YELLOW");
			Sound.playTone(1000, weight);
			Sound.playTone(1000, weight);
			Sound.playTone(1000, weight);
			return 3;
		}
		else {
			System.out.println("BLUE");
			Sound.playTone(1000, weight);
			return 1;
		}
	
	}
	
	/**
	 * Computes the mean RGB Values returned for the can
	 */
	private void computeMean() {
		double sumRed = 0;
		double sumBlue = 0;
		double sumGreen = 0;
		
		double currentRed;
		double currentBlue;
		double currentGreen;

		for (int i = 0; i < 10; i++) {
			double normalize = Math.sqrt(Math.pow(redArray[i], 2) + Math.pow(greenArray[i],2) + Math.pow(greenArray[i],2));
			currentRed= redArray[i];
			sumRed += (currentRed/normalize);

			currentGreen =greenArray[i];
			sumGreen += (currentGreen/normalize);

			currentBlue = blueArray[i];
			sumBlue += (currentBlue/normalize);
		}
		
		//compute the non-normalized mean
		redMean = sumRed/10;
		greenMean = sumGreen/10;
		blueMean = sumBlue/10;
		
	}
	
}
