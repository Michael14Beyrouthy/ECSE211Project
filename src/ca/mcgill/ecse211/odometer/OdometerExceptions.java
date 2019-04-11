package ca.mcgill.ecse211.odometer;
/**
 * This class is used to handle errors regarding the singleton pattern used for the odometer and
 * odometerData
 * @author Michael Beyrouthy
 */
@SuppressWarnings("serial")
public class OdometerExceptions extends Exception {

	/**
	 * Constructor for the class
	 * @param Error
	 */
	public OdometerExceptions(String Error) {
		super(Error);
	}
}
