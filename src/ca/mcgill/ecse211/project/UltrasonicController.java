package ca.mcgill.ecse211.project;

/**
 * UltrasonicController interface, declares methods used by classes using the US sensor
 * @author Hongshuo Zhou
 */
public interface UltrasonicController {

	  public void processUSData(int distance);

	  public int readUSDistance();
	}