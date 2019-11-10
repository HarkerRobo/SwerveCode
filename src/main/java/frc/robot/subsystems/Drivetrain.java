package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SwervePercentOutput;
import frc.robot.util.SwerveModule;
import frc.robot.util.Vector;
import harkerrobolib.wrappers.HSTalon;

import java.util.function.Consumer;

import com.ctre.phoenix.sensors.PigeonIMU;

/**
 * Simulates the drivetrain subsystem on the robot. 
 * A Swerve Drivetrain contains four SwerveModules.
 * 
 * Acronyms:
 *      TL: Top Left
 *      TR: Top Right
 *      BL: Bottom Left
 *      BR: Bottom Right
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @since 11/1/19
 */
public class Drivetrain extends Subsystem {

	public static Drivetrain instance;

    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    
    private boolean isFieldSensitive;
    private PigeonIMU pigeon;

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

	public static final double DT_WIDTH = 16.5; //Inches between each back wheel
    public static final double DT_LENGTH = 20.6; //Inches between each the back and front wheels
    
    public static final int TL_OFFSET = 1637;
    public static final int TR_OFFSET = 6846;
    public static final int BL_OFFSET = 11239;
    private static final int BR_OFFSET = 4520;

	private Drivetrain() {
		topLeft = new SwerveModule(RobotMap.TL_DRIVE_ID, TL_DRIVE_INVERTED, TL_DRIVE_SENSOR_PHASE, RobotMap.TL_ANGLE_ID,                TL_ANGLE_INVERTED, TL_ANGLE_SENSOR_PHASE);
		topRight = new SwerveModule(RobotMap.TR_DRIVE_ID, TR_DRIVE_INVERTED, TR_DRIVE_SENSOR_PHASE, RobotMap.TR_ANGLE_ID,               TR_ANGLE_INVERTED, TR_ANGLE_SENSOR_PHASE);
		backLeft = new SwerveModule(RobotMap.BL_DRIVE_ID, BL_DRIVE_SENSOR_PHASE, BL_DRIVE_INVERTED, RobotMap.BL_ANGLE_ID,               BL_ANGLE_INVERTED, BL_ANGLE_SENSOR_PHASE);
		backRight = new SwerveModule(RobotMap.BR_DRIVE_ID, BR_DRIVE_INVERTED, BR_DRIVE_SENSOR_PHASE, RobotMap.BR_ANGLE_ID,              BR_ANGLE_INVERTED, BR_ANGLE_SENSOR_PHASE);

        isFieldSensitive = false;
        pigeon = new PigeonIMU(RobotMap.PIGEON_ID);
        pigeon.setCompassAngle(0);

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
    }
    
    public void setupPositionPID() {
        applyToAll((module) -> module.getAngleMotor().config_kP(ANGLE_POSITION_SLOT, ANGLE_POSITION_KP));
        applyToAll((module) -> module.getAngleMotor().config_kI(ANGLE_POSITION_SLOT, ANGLE_POSITION_KI));
        applyToAll((module) -> module.getAngleMotor().config_kD(ANGLE_POSITION_SLOT, ANGLE_POSITION_KD));
	}
	
    public void setupVelocityPID() {
		applyToAll((module) -> module.getDriveMotor().config_kP(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KP));
        applyToAll((module) -> module.getDriveMotor().config_kI(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KI));
        applyToAll((module) -> module.getDriveMotor().config_kD(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KD));
        applyToAll((module) -> module.getDriveMotor().config_kF(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KF));
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
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new SwervePercentOutput());
    }
    
	/**
	 * Sets the output of the drivetrain based on sum vectors for each swerve module
	 */
	public void setDrivetrain(Vector tr, Vector tl, Vector br, Vector bl) {
		setSwerveModule(topRight, tr.getMagnitude(), convertAngle(topRight, tr.getAngle()));
		setSwerveModule(topLeft, tl.getMagnitude(), convertAngle(topLeft, tl.getAngle()));
		setSwerveModule(backRight, br.getMagnitude(), convertAngle(backRight, br.getAngle()));
		setSwerveModule(backLeft, bl.getMagnitude(), convertAngle(backLeft, bl.getAngle()));
	}

	/** Ensures that the swerve module never has to turn more than 90 degrees, by inverting the 
	 * motors if needed. For example, if we are turning from 0 to 180 degrees, we can just not
	 * turn and invert the motor. 
	 *    
	 */
    public double convertAngle(SwerveModule module, double targetAngle) {

        targetAngle -= 90;
        double currDegrees = module.getAngleDegrees();

        while (currDegrees - targetAngle > 180) {
            targetAngle += 360;
        } 
        while (currDegrees - targetAngle < -180) {
            targetAngle -= 360;
        }
        
		if (Math.abs(module.getAngleDegrees() - targetAngle) > 90) {
			module.invertOutput();
			targetAngle += 180;
        } 	
        
        // if(isFieldSensitive)
        //     targetAngle -= pigeon.getAbsoluteCompassHeading();

		return (targetAngle);
	}

	public void setSwerveModule(SwerveModule module, double output, double angle) {
        module.setTargetAngle(angle);
        module.setOutput(output);
	}

	public void stopDrivetrain() {
		setSwerveModule(topRight, 0, 0);
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
	
	public static Drivetrain getInstance() {
		if(instance == null) 
			instance = new Drivetrain();
		return instance;
	}
}
