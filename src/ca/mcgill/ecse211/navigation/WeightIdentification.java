package ca.mcgill.ecse211.navigation;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;

public class WeightIdentification {
	
	/**
	 * Power for unregulated weighing motor
	 */
	private static final int MOTOR_POWER = 12;
	
	/**
	 * Threshold tachomter count difference to classify cans as either heavy or light
	 */
	private static final int TACHO_THRESHOLD = 20;

	/**
	 * Classifies a can as either heavy or light
	 * @param clawMotor The motor used to grasp cans
	 * @return 500mS for a light can, 1000mS for a heavy can
	 */
	public EV3LargeRegulatedMotor getWeight(EV3LargeRegulatedMotor clawMotor) {
		
		//close clawMotor instance
		clawMotor.close();
		
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
		
		// Re-instantiate claw motor as a regulated motor
		//clawMotor= new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
		
		// Get difference in tachometer count after pushing can
		int weight = Math.abs(secondTacho-firstTacho);
		
		if(weight>=TACHO_THRESHOLD) {
			System.out.println("LIGHT");
			return new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
		}
		else {
			System.out.println("HEAVY");
			return new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
		}
	}
}
