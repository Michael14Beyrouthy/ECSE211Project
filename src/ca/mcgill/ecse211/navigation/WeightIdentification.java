package ca.mcgill.ecse211.navigation;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.UnregulatedMotor;

public class WeightIdentification {
	
	/**
	 * Power for unregulated weighing motor
	 */
	private static final int MOTOR_POWER = 15;
	
	/**
	 * Threshold tachomter count difference to classify cans as either heavy or light
	 */
	private static final int TACHO_THRESHOLD = 20;

	/**
	 * Classifies a can as either heavy or light
	 * @return 500mS for a light can, 1000mS for a heavy can
	 */
	public int getWeight() {
		
		
		//Re-initialize the claw motor as an unregulated motor
		UnregulatedMotor weighingMotor=new UnregulatedMotor(LocalEV3.get().getPort("B"));
		
		int firstTacho = weighingMotor.getTachoCount();
		
		weighingMotor.setPower(MOTOR_POWER);
		weighingMotor.backward(); // Push the can with the weighing motor
		
		try { //Wait 1 second for motor to push can
			Thread.sleep(1000);
	    	} catch (InterruptedException e) {
	    	}
		
		weighingMotor.stop();
		int secondTacho = weighingMotor.getTachoCount();
		
		//Move claw back to it's original position
		weighingMotor.forward();
		try {
			Thread.sleep(1000);
	    	} catch (InterruptedException e) {
	      // There is nothing to be done here
	    	}
		weighingMotor.stop();
		
		
		System.out.println("first: " + firstTacho);
		System.out.println("second: " + secondTacho);
		
		// Close weighing motor instance
		weighingMotor.close();
		
		// Get difference in tachometer count after pushing can
		int weight = Math.abs(secondTacho-firstTacho);
		
		if(weight>=TACHO_THRESHOLD) {
			System.out.println("LIGHT");
			return 500;
		}
		else {
			System.out.println("HEAVY");
			return 1000;
		}
	}
}
