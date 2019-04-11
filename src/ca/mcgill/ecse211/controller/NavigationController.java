package ca.mcgill.ecse211.controller;

/**
 * NavigationController class, holds certain robot constants
 * @author Xiangyu Li
 */
public interface NavigationController {

		// vehicle hardware parameters
		public static final double WHEEL_RADIUS = 2.09;	// (cm)
		public static final double TRACK = 14.5;		//(cm)
		public static final double TILE_LENGTH = 30.48;	//(cm)
		// motor parameters
		public static final int FORWARD_SPEED = 250;
		public static final int CLAW_SPEED = 150;
		public static final int CORRECTION_SPEED = 150;
		
		//parameters for search
		public static final double CAN_DIST = 22;
		
		public static final double COLOR = 0.30;
		
		public static final long MAXNUMBERCANS = 2;

}
