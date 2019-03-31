package ca.mcgill.ecse211.controller;

/**
 * NavigationController class, holds certain robot constants
 * @author Sumail
 */
public interface NavigationController {

		// vehicle hardware parameters
		public static final double WHEEL_RADIUS = 2.20;	// (cm)
		public static final double TRACK = 14.5;		//(cm)
		public static final double TILE_LENGTH = 30.48;	//(cm)
		// motor parameters
		public static final int FORWARD_SPEED = 300;
		public static final int ROTATE_SPEED = 250;
		// parameters used for bang b\ang controller in navigation avoid
		public static final int OBSTACLE_LENGTH = 20;
		public static final int OBSTACLE_WIDTH = 35;


}
