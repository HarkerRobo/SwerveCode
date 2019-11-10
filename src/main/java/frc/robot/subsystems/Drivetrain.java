package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SwervePercentOutput;
import frc.robot.util.SwerveModule;
import frc.robot.util.Vector;
import harkerrobolib.wrappers.HSPigeon;
import harkerrobolib.wrappers.HSTalon;

import java.util.function.Consumer;

/**
 * Simulates the drivetrain subsystem on the robot. 
 * A Swerve Drivetrain contains four SwerveModules.
 * 
 * 'back' is defined as closest to the battery
 * 'left' is defined as left when standing at the back and looking forward
 * 
 * Acronyms:
 *      TL: Top Left
 *      TR: Top Right
 *      BL: Back Left
 *      BR: Back Right
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Arjun Dixit
 * @since 11/1/19
 */
public class Drivetrain extends Subsystem {

	public static Drivetrain instance;

    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    
    private boolean isFieldSensitive;
    private HSPigeon pigeon;

	private static final boolean TL_DRIVE_INVERTED = true;
	private static final boolean TL_ANGLE_INVERTED = false;
	private static final boolean TR_DRIVE_INVERTED = true;
	private static final boolean TR_ANGLE_INVERTED = true;
    private static final boolean BL_DRIVE_INVERTED = true;
	private static final boolean BL_ANGLE_INVERTED = true;
	private static final boolean BR_DRIVE_INVERTED = true;
    private static final boolean BR_ANGLE_INVERTED = true;
    
	private static final boolean TL_DRIVE_SENSOR_PHASE = true;
	private static final boolean TL_ANGLE_SENSOR_PHASE = false;
	private static final boolean TR_DRIVE_SENSOR_PHASE = true;
	private static final boolean TR_ANGLE_SENSOR_PHASE = true;
    private static final boolean BL_DRIVE_SENSOR_PHASE = true;
	private static final boolean BL_ANGLE_SENSOR_PHASE = true;
	private static final boolean BR_DRIVE_SENSOR_PHASE = true;
	private static final boolean BR_ANGLE_SENSOR_PHASE = true;

    public static final int ANGLE_POSITION_SLOT = 0;
	private static final double ANGLE_POSITION_KP = 0.3;
	private static final double ANGLE_POSITION_KI = 0.0;
	private static final double ANGLE_POSITION_KD = 0.0;

	public static final int DRIVE_VELOCITY_SLOT = 0;
	private static final double DRIVE_VELOCITY_KP = 0.0;
	private static final double DRIVE_VELOCITY_KI = 0.0;
	private static final double DRIVE_VELOCITY_KD = 0.0;
	private static final double DRIVE_VELOCITY_KF = 0.0;

    /**
     * Inches between both of the wheels on the front or back
     */
    public static final double DT_WIDTH = 16.5;
    /**
     * Inches between both of the wheels on the left or right
     */
    public static final double DT_LENGTH = 20.6; 
    
    public static final int TL_OFFSET = 1637;
    public static final int TR_OFFSET = 6846;
    public static final int BL_OFFSET = 11239;
    private static final int BR_OFFSET = 4520;

	private Drivetrain() {
		topLeft = new SwerveModule(RobotMap.TL_DRIVE_ID, TL_DRIVE_INVERTED, TL_DRIVE_SENSOR_PHASE, RobotMap.TL_ANGLE_ID, TL_ANGLE_INVERTED, TL_ANGLE_SENSOR_PHASE);
		topRight = new SwerveModule(RobotMap.TR_DRIVE_ID, TR_DRIVE_INVERTED, TR_DRIVE_SENSOR_PHASE, RobotMap.TR_ANGLE_ID, TR_ANGLE_INVERTED, TR_ANGLE_SENSOR_PHASE);
		backLeft = new SwerveModule(RobotMap.BL_DRIVE_ID, BL_DRIVE_INVERTED, BL_DRIVE_SENSOR_PHASE, RobotMap.BL_ANGLE_ID, BL_ANGLE_INVERTED, BL_ANGLE_SENSOR_PHASE);
		backRight = new SwerveModule(RobotMap.BR_DRIVE_ID, BR_DRIVE_INVERTED, BR_DRIVE_SENSOR_PHASE, RobotMap.BR_ANGLE_ID, BR_ANGLE_INVERTED, BR_ANGLE_SENSOR_PHASE);

        int tlAngleOffset = (topLeft.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - TL_OFFSET) / 4;
        int trAngleOffset = (topRight.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - TR_OFFSET) / 4;
        int blAngleOffset = (backLeft.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - BL_OFFSET) / 4;
        int brAngleOffset = (backRight.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - BR_OFFSET) / 4;

        topLeft.getAngleMotor().setSelectedSensorPosition(tlAngleOffset);
        topRight.getAngleMotor().setSelectedSensorPosition(trAngleOffset);
        backLeft.getAngleMotor().setSelectedSensorPosition(blAngleOffset);
        backRight.getAngleMotor().setSelectedSensorPosition(brAngleOffset);

        setupPositionPID();
        setupVelocityPID();
        
        isFieldSensitive = false;
        pigeon = new HSPigeon(RobotMap.PIGEON_ID);
        pigeon.configFactoryDefault();
        pigeon.zero();
    }

    @Override
	protected void initDefaultCommand() {
		setDefaultCommand(new SwervePercentOutput());
    }
    
    public void setupPositionPID() {
        applyToAllAngle((angleMotor) -> angleMotor.config_kP(ANGLE_POSITION_SLOT, ANGLE_POSITION_KP));
        applyToAllAngle((angleMotor) -> angleMotor.config_kI(ANGLE_POSITION_SLOT, ANGLE_POSITION_KI));
        applyToAllAngle((angleMotor) -> angleMotor.config_kD(ANGLE_POSITION_SLOT, ANGLE_POSITION_KD));
	}
	
    public void setupVelocityPID() {
        applyToAllDrive((driveMotor) -> driveMotor.config_kF(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KF));
		applyToAllDrive((driveMotor) -> driveMotor.config_kP(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KP));
        applyToAllDrive((driveMotor) -> driveMotor.config_kI(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KI));
        applyToAllDrive((driveMotor) -> driveMotor.config_kD(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KD));
	}
    
	/**
	 * Sets the output of the drivetrain based on desired output vectors for each swerve module
	 */
	public void setDrivetrain(Vector tl, Vector tr, Vector bl, Vector br) {
		setSwerveModule(topLeft, tl.getMagnitude(), convertAngle(topLeft, tl.getAngle()));
		setSwerveModule(topRight, tr.getMagnitude(), convertAngle(topRight, tr.getAngle()));
		setSwerveModule(backLeft, bl.getMagnitude(), convertAngle(backLeft, bl.getAngle()));
		setSwerveModule(backRight, br.getMagnitude(), convertAngle(backRight, br.getAngle()));
	}

	/** 
     * Converts the target angle from the desired Vectors into the actual angle for the motors to hold.
     * 
     * Steps:
     * 
     * 1. Subtract 90 degrees. 0 degrees on the Joysticks and desired Vectors points to the right (positive x axis)
     *    while 0 degrees on the robot points forward (positive y axis). The subtraction deals with this offset.
     * 2. Increase/Decrease the targetAngle by 360 degrees until it is within +- 180 degrees of the current angle
     * 3. Ensure the angle motor does not have to turn more than 90 degrees 
     *    (if we need to turn 179 degrees, we can instead turn -1 degree and invert output)
     * 4. If using Field Sensitive Mode, ensure the direction of 0 degrees is relative to the field
     *    rather than the robot's orientation    
	 */
    public double convertAngle(SwerveModule module, double targetAngle) {
        //Step 1
        targetAngle -= 90; 

        double currDegrees = module.getAngleDegrees();

        //Step 2
        while (currDegrees - targetAngle > 180) {
            targetAngle += 360;
        } 
        while (currDegrees - targetAngle < -180) {
            targetAngle -= 360;
        }
        
        //Step 3
		if (currDegrees - targetAngle > 90) {
			module.invertOutput();
			targetAngle += 180;
        }
        else if (currDegrees - targetAngle < -90) {
            module.invertOutput();
            targetAngle -= 180;
        }
        
        //Step 4
        if(isFieldSensitive) {
            targetAngle -= pigeon.getYaw(); //Postive Yaw = Counter-clockwise turning
        }

		return (targetAngle);
	}

	public void setSwerveModule(SwerveModule module, double output, double angle) {
        module.setTargetAngle(angle);
        module.setOutputPercent(output);
	}

    /**
     * Stops all drive motors while holding the current angle
     */
	public void stopAllDrive() {
        setSwerveModule(topLeft, 0, topLeft.getAngleDegrees());
        setSwerveModule(topRight, 0, topRight.getAngleDegrees());
        setSwerveModule(backLeft, 0, backLeft.getAngleDegrees());
        setSwerveModule(backRight, 0, backRight.getAngleDegrees());
	}
	
	/**
	 * Calls a method on all swerve modules on the drivetrain. For example:
	 * Drivetrain.getInstance().applyToAll((motor) -> motor.doSomething());
	 */
	public void applyToAll(Consumer<SwerveModule> consumer) {
		consumer.accept(topLeft);
		consumer.accept(topRight);
		consumer.accept(backLeft);
		consumer.accept(backRight);
	}

	/**
	 * Calls a method on the angle motor of each swerve module.
	 */
	public void applyToAllAngle(Consumer<HSTalon> consumer) {
		consumer.accept(topLeft.getAngleMotor());
		consumer.accept(topRight.getAngleMotor());
		consumer.accept(backLeft.getAngleMotor());
		consumer.accept(backRight.getAngleMotor());
	}

	/**
	 * Calls a method on the drive motor of each swerve module.
	 */
	public void applyToAllDrive(Consumer<HSTalon> consumer) {
		consumer.accept(topLeft.getDriveMotor());
		consumer.accept(topRight.getDriveMotor());
		consumer.accept(backLeft.getDriveMotor());
		consumer.accept(backRight.getDriveMotor());
    }
	
    public SwerveModule getTopLeft() {
		return topLeft;
	}

	public SwerveModule getTopRight() {
		return topRight;
	}

	public SwerveModule getBackLeft() {
		return backLeft;
	}

	public SwerveModule getBackRight() {
		return backRight;
    }
    
    public HSPigeon getPigeon() {
        return pigeon;
    }

    public static Drivetrain getInstance() {
		if(instance == null) 
			instance = new Drivetrain();
		return instance;
    }
}
