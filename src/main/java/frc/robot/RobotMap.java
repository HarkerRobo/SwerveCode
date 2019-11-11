package frc.robot;

/**
 * Contains project-wide constants, including CAN IDs
 * 
 * Acronyms:
 *      TL: Top Left
 *      TR: Top Right
 *      BL: Bottom Left
 *      BR: Bottom Right
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Anirudh Kotamraju
 * @author Shahzeb Lakhani
 * @author Arjun Dixit
 * @since 11/11/19
 */
public class RobotMap {

    public static final int PRIMARY_INDEX = 0;
    public static final int AUXILIARY_INDEX = 1;
    
    //Drivetrain CAN ids.
    public static final int[] DRIVE_IDS = {1, 7, 6, 4};
    public static final int[] ANGLE_IDS = {0, 5, 3, 2};

	public static final int PIGEON_ID = 1;
}