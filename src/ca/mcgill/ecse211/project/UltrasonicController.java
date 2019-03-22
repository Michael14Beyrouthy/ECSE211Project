package ca.mcgill.ecse211.project;

public interface UltrasonicController {

	  public void processUSData(int distance);

	  public int readUSDistance();
	}