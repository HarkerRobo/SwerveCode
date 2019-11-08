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
 * @since 11/1/19
 */
public class RobotMap {

    public static final int PRIMARY_INDEX = 0;
    public static final int AUXILIARY_INDEX = 1;
    
    //Drivetrian CAN ids.
    public static final int TL_DRIVE_ID = 1;
    public static final int TL_ANGLE_ID = 0;

    public static final int TR_DRIVE_ID = 7;
    public static final int TR_ANGLE_ID = 5;

    public static final int BL_DRIVE_ID = 6;
    public static final int BL_ANGLE_ID = 3;

    public static final int BR_DRIVE_ID = 4;
    public static final int BR_ANGLE_ID = 2;

}