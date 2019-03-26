package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.project.UltrasonicController;
import lejos.robotics.SampleProvider;

/*
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */

/**
 * UltrasonicPoller class, periodically processes the distance from an object
 * @author Sumail
 *
 */
public class UltrasonicPoller extends Thread {
  private SampleProvider us;
  private UltrasonicController cont;
  private float[] usData;

  /**
   * Constructor for the class
   * @param us
   * @param usData
   * @param cont
   */
  public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicController cont) {
    this.us = us;
    this.cont = cont;
    this.usData = usData;
  }
  /**
   * Another constructor for the class
   * @param distanceMode
   */
  public UltrasonicPoller(SampleProvider distanceMode) {
	// TODO Auto-generated constructor stub
}

/*
   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
   * [0,255] (non-Javadoc)
   * 
   * @see java.lang.Thread#run()
   */
  
  /**
   * run() method for thread
   */
  public void run() {
    int distance;
    while (true) {
      us.fetchSample(usData, 0); // acquire data
      distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
      cont.processUSData(distance); // now take action depending on value
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      } 
    }
  }

}
