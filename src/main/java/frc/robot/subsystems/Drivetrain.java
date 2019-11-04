package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;

/**
 * Simulates the drivetrain subsystem on the robot. 
 * 
 * @version 11/3/19
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 */
public class Drivetrain extends Subsystem {

	public static Drivetrain instance;

    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule bottomLeft;
	private SwerveModule bottomRight;

	private static final boolean TOP_LEFT_DRIVE_INVERTED = false;
	private static final boolean TOP_LEFT_ANGLE_INVERTED = false;
	private static final boolean TOP_RIGHT_DRIVE_INVERTED = false;
	private static final boolean TOP_RIGHT_ANGLE_INVERTED = false;
    private static final boolean BOTTOM_LEFT_DRIVE_INVERTED = false;
	private static final boolean BOTTOM_LEFT_ANGLE_INVERTED = false;
	private static final boolean BOTTOM_RIGHT_DRIVE_INVERTED = false;
	private static final boolean BOTTOM_RIGHT_ANGLE_INVERTED = false;
	
	private static final boolean TOP_LEFT_DRIVE_SENSOR_PHASE = false;
	private static final boolean TOP_LEFT_ANGLE_SENSOR_PHASE = false;
	private static final boolean TOP_RIGHT_DRIVE_SENSOR_PHASE = false;
	private static final boolean TOP_RIGHT_ANGLE_SENSOR_PHASE = false;
    private static final boolean BOTTOM_LEFT_DRIVE_SENSOR_PHASE = false;
	private static final boolean BOTTOM_LEFT_ANGLE_SENSOR_PHASE = false;
	private static final boolean BOTTOM_RIGHT_DRIVE_SENSOR_PHASE = false;
	private static final boolean BOTTOM_RIGHT_ANGLE_SENSOR_PHASE = false;

	private Drivetrain() {
		topLeft = new SwerveModule(RobotMap.TOP_LEFT_DRIVE_ID, TOP_LEFT_DRIVE_INVERTED, TOP_LEFT_DRIVE_SENSOR_PHASE, RobotMap.TOP_LEFT_ANGLE_ID, TOP_LEFT_ANGLE_INVERTED, TOP_LEFT_ANGLE_SENSOR_PHASE);
		topRight = new SwerveModule(RobotMap.TOP_RIGHT_DRIVE_ID, TOP_RIGHT_DRIVE_INVERTED, TOP_RIGHT_DRIVE_SENSOR_PHASE, RobotMap.TOP_RIGHT_ANGLE_ID, TOP_RIGHT_ANGLE_INVERTED, TOP_RIGHT_ANGLE_SENSOR_PHASE);
		bottomLeft = new SwerveModule(RobotMap.BOTTOM_LEFT_DRIVE_ID, BOTTOM_LEFT_DRIVE_SENSOR_PHASE, BOTTOM_LEFT_DRIVE_INVERTED, RobotMap.BOTTOM_LEFT_ANGLE_ID, BOTTOM_LEFT_ANGLE_INVERTED, BOTTOM_LEFT_ANGLE_SENSOR_PHASE);
		bottomRight = new SwerveModule(RobotMap.BOTTOM_RIGHT_DRIVE_ID, BOTTOM_RIGHT_DRIVE_INVERTED, BOTTOM_RIGHT_DRIVE_SENSOR_PHASE, RobotMap.BOTTOM_RIGHT_ANGLE_ID, BOTTOM_RIGHT_ANGLE_INVERTED, BOTTOM_RIGHT_ANGLE_SENSOR_PHASE);
	}

	public SwerveModule getTopLeft() {
		return topLeft;
	}

	public SwerveModule getTopRight() {
		return topRight;
	}

	public SwerveModule getBottomLeft() {
		return bottomLeft;
	}

	public SwerveModule getBottomRight() {
		return bottomRight;
	}
	
	@Override
	protected void initDefaultCommand() {
		
	}
	
	public static Drivetrain getInstance() {
		if(instance == null) instance = new Drivetrain();
		return instance;
	}
}