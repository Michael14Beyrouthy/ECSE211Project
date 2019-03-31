package ca.mcgill.ecse211.navigation;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;

public class WeightIdentification {

	EV3LargeRegulatedMotor clawMotor;
	
	public WeightIdentification(EV3LargeRegulatedMotor clawMotor) {
		this.clawMotor=clawMotor;
	}
	

	public int getWeight() {
		clawMotor.close();
		UnregulatedMotor testMotor=new UnregulatedMotor(LocalEV3.get().getPort("B"));
		int firstTacho = testMotor.getTachoCount();
		testMotor.setPower(12);
		testMotor.backward();
		try {
			Thread.sleep(1000);
	    	} catch (InterruptedException e) {
	      // There is nothing to be done here
	    	}
		testMotor.stop();
		testMotor.forward();
		try {
			Thread.sleep(1000);
	    	} catch (InterruptedException e) {
	      // There is nothing to be done here
	    	}
		testMotor.stop();
		int secondTacho = testMotor.getTachoCount();
		System.out.println("first: " + firstTacho);
		System.out.println("second: " + secondTacho);
		testMotor.close();
		int weight = Math.abs(secondTacho-firstTacho);
		if(weight>=20) {
			System.out.println("LIGHT");
		}
		else {
			System.out.println("HEAVY");
		}
		clawMotor= new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
		return 1;
	}
}
