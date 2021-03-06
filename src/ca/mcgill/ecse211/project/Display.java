package ca.mcgill.ecse211.project;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;

/**
 * Display class, displays odometer's values (x, y, Theta)
 * @author Jamie McLeish
 */
public class Display implements Runnable {

  private Odometer odo;
  private TextLCD lcd;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * Class constructor
   * @param lcd
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd, Odometer odo) throws OdometerExceptions {
    this.odo = odo;//Odometer.getOdometer();
    this.lcd = lcd;
  }

  /**
   * Overloaded class constructor
   * @param lcd
   * @param timeout
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd, long timeout, Odometer odo) throws OdometerExceptions {
    this.odo = odo;//Odometer.getOdometer();
    this.timeout = timeout;
    this.lcd = lcd;
  }

  /**
   * run() method for thread
   */
  public void run() {
    
    lcd.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odo.getXYT();
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      
      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

}