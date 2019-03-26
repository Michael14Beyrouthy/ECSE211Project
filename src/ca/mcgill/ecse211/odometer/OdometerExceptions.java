package ca.mcgill.ecse211.odometer;

/**
 * OdometerExceptions class, used to handle errors regarding the singleton pattern used for the odometer and
 * odometerData
 */
@SuppressWarnings("serial")
public class OdometerExceptions extends Exception {

  /**
   * Class constructor
   * @param Error
   */
  public OdometerExceptions(String Error) {
    super(Error);
  }

}
