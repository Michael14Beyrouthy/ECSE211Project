package ca.mcgill.ecse211.navigation;

/**
 * NavigationController class, holds certain robot constants 
 * @author Sumail
 *
 */
public interface NavigationController {

		// vehicle hardware parameters
		public static final double WHEEL_RADIUS = 2.09;	// (cm)
		public static final double TRACK = 14.13;		//(cm)
		public static final double TILE_LENGTH = 30.48;	//(cm)
		// motor parameters
		public static final int FORWARD_SPEED = 250;
		public static final int ROTATE_SPEED = 150;
		// parameters used for bang b\ang controller in navigation avoid
		public static final int OBSTACLE_LENGTH = 20;
		public static final int OBSTACLE_WIDTH = 35;
		
		


}
