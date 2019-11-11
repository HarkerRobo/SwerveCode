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

    private SwerveModule[] moduleArr;
    
    private boolean isFieldSensitive;
    private HSPigeon pigeon;

    private static final boolean[] DRIVE_INVERTED = {true, true, true, true};
    private static final boolean[] ANGLE_INVERTED = {false, true, true, true};

    private static final boolean[] DRIVE_SENSOR_PHASE_ARR = {true, true, true, true};
    private static final boolean[] ANGLE_SENSOR_PHASE_ARR = {false, true, true, true};

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
    
    public static final int[] OFFSETS = {1637, 6846, 11239, 4520};

	private Drivetrain() {
        
        for(int i = 0; i < 4; i++) {
            moduleArr[i] = new SwerveModule(RobotMap.DRIVE_IDS[i], DRIVE_INVERTED[i], DRIVE_SENSOR_PHASE_ARR[i], 
                                            RobotMap.ANGLE_IDS[i], ANGLE_INVERTED[i], ANGLE_SENSOR_PHASE_ARR[i]);
            HSTalon angleMotor = moduleArr[i].getAngleMotor();
            angleMotor.setSelectedSensorPosition((angleMotor.getSensorCollection().getPulseWidthRiseToFallUs() - OFFSETS[i]) / 4);
        }

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
	public void setDrivetrain(Vector[] vecArr) {
        for(int i = 0; i < 4; i++)
            setSwerveModule(moduleArr[i], vecArr[i].getMagnitude(), convertAngle(moduleArr[i], vecArr[i].getAngle()));
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
        for(SwerveModule module : moduleArr)
            setSwerveModule(module, 0, module.getAngleDegrees());
	}
	
	/**
	 * Calls a method on all swerve modules on the drivetrain. For example:
	 * Drivetrain.getInstance().applyToAll((motor) -> motor.doSomething());
	 */
	public void applyToAll(Consumer<SwerveModule> consumer) {
        for(SwerveModule module : moduleArr)
		    consumer.accept(module);
	}

	/**
	 * Calls a method on the angle motor of each swerve module.
	 */
	public void applyToAllAngle(Consumer<HSTalon> consumer) {
        for(SwerveModule module : moduleArr)
		    consumer.accept(module.getAngleMotor());
	}

	/**
	 * Calls a method on the drive motor of each swerve module.
	 */
	public void applyToAllDrive(Consumer<HSTalon> consumer) {
		for(SwerveModule module : moduleArr)
		    consumer.accept(module.getDriveMotor());
    }

    public SwerveModule[] getModuleArr()
    {
        return moduleArr;
    }
	
    public SwerveModule getTopLeft() {
		return moduleArr[0];
	}

	public SwerveModule getTopRight() {
		return moduleArr[1];
	}

	public SwerveModule getBackLeft() {
		return moduleArr[2];
	}

	public SwerveModule getBackRight() {
		return moduleArr[3];
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
