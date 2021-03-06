package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SwerveManual;
import frc.robot.util.SwerveModule;
import frc.robot.util.Vector;
import harkerrobolib.util.Conversions;
import harkerrobolib.wrappers.HSPigeon;
import harkerrobolib.wrappers.HSTalon;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.StatusFrame;

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

    private static final boolean TL_DRIVE_INVERTED = false;
    private static final boolean TL_ANGLE_INVERTED = false;
    private static final boolean TR_DRIVE_INVERTED = false;
    private static final boolean TR_ANGLE_INVERTED = true;
    private static final boolean BL_DRIVE_INVERTED = false;
    private static final boolean BL_ANGLE_INVERTED = true;
    private static final boolean BR_DRIVE_INVERTED = false;
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
    private static final double ANGLE_POSITION_KP = 0.85;//1.0
    private static final double ANGLE_POSITION_KI = 0.0;
    private static final double ANGLE_POSITION_KD = 30.0;

    public static final int DRIVE_VELOCITY_SLOT = 0;
    private static final double DRIVE_VELOCITY_KP = 0.13;
    private static final double DRIVE_VELOCITY_KI = 0.0;
    private static final double DRIVE_VELOCITY_KD = 0.0;
    private static final double DRIVE_VELOCITY_KF = 0.046;// theoretical:  0.034;

    public static final int DRIVE_MOTION_PROF_SLOT = 1;
    public static final double DRIVE_MOTION_PROF_kF = DRIVE_VELOCITY_KF;
    private static final double DRIVE_MOTION_PROF_kP = 0.3;
    private static final double DRIVE_MOTION_PROF_kI = 0;
    private static final double DRIVE_MOTION_PROF_kD = 1;
    public static final double DRIVE_MOTION_PROF_kS = 0.06;

    public static final int ANGLE_MOTION_PROF_SLOT = 1;
    private static final double ANGLE_MOTION_PROF_kF = 0;
    private static final double ANGLE_MOTION_PROF_kP = ANGLE_POSITION_KP;
    private static final double ANGLE_MOTION_PROF_kI = 0;
    private static final double ANGLE_MOTION_PROF_kD = 0;

    public static final double MOTION_PROF_RAMP_RATE = 0.3;

    private static final int MOTION_FRAME_PERIOD = 5;
  
    public static final double MAX_DRIVE_VELOCITY = 9;
    public static final double MAX_DRIVE_ACCELERATION = 4;
    public static final double MAX_DRIVE_JERK = 50;
    
    public static final double DRIVE_RAMP_RATE = 0.1;
    public static final double ANGLE_RAMP_RATE = 0.05;
    public static final double GEAR_RATIO = 6;
    
    /**
     * Meters between both of the wheels on the front or back
     */
    public static final double DT_WIDTH = 0.419; //16.5 in 
    /**
     * Meters between both of the wheels on the left or right
     */
    public static final double DT_LENGTH = 0.523; //20.6 in
    
    public static final int TL_OFFSET = 2212;//1749;//2212-1749
    public static final int TR_OFFSET = 6730;//7638;
    public static final int BL_OFFSET = 11327;
    private static final int BR_OFFSET = 4605;

	public static final double PIGEON_kP = 0.02;

    private static final double WHEEL_DIAMETER = 4;

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
        setupMotionProfilePID();
        
        isFieldSensitive = true;
        pigeon = new HSPigeon(RobotMap.PIGEON_ID);
        pigeon.configFactoryDefault();
        pigeon.zero();

        Conversions.setWheelDiameter(WHEEL_DIAMETER);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SwerveManual());
    }

    public void toggleFieldSensitivity() {
        isFieldSensitive = !isFieldSensitive;
        System.out.println(isFieldSensitive);
    }
    

    public void setupPositionPID() {
        applyToAllAngle((angleMotor) -> angleMotor.config_kP(ANGLE_POSITION_SLOT, ANGLE_POSITION_KP));
        applyToAllAngle((angleMotor) -> angleMotor.config_kI(ANGLE_POSITION_SLOT, ANGLE_POSITION_KI));
        applyToAllAngle((angleMotor) -> angleMotor.config_kD(ANGLE_POSITION_SLOT, ANGLE_POSITION_KD));
        applyToAllAngle((angleMotor) -> angleMotor.configClosedloopRamp(ANGLE_RAMP_RATE));
    }
    
    public void setupVelocityPID() {
        applyToAllDrive((driveMotor) -> driveMotor.config_kF(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KF));
        applyToAllDrive((driveMotor) -> driveMotor.config_kP(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KP));
        applyToAllDrive((driveMotor) -> driveMotor.config_kI(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KI));
        applyToAllDrive((driveMotor) -> driveMotor.config_kD(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KD));
        applyToAllDrive((driveMotor) -> driveMotor.configClosedloopRamp(DRIVE_RAMP_RATE));
    }
    
    private void setupMotionProfilePID() {
        applyToAllDrive((driveMotor) -> driveMotor.config_kF(DRIVE_MOTION_PROF_SLOT, DRIVE_MOTION_PROF_kF));
        applyToAllDrive((driveMotor) -> driveMotor.config_kP(DRIVE_MOTION_PROF_SLOT, DRIVE_MOTION_PROF_kP));
        applyToAllDrive((driveMotor) -> driveMotor.config_kI(DRIVE_MOTION_PROF_SLOT, DRIVE_MOTION_PROF_kI));
        applyToAllDrive((driveMotor) -> driveMotor.config_kD(DRIVE_MOTION_PROF_SLOT, DRIVE_MOTION_PROF_kD));

        applyToAllAngle((angleMotor) -> angleMotor.config_kF(ANGLE_MOTION_PROF_SLOT, ANGLE_MOTION_PROF_kF));
        applyToAllAngle((angleMotor) -> angleMotor.config_kP(ANGLE_MOTION_PROF_SLOT, ANGLE_MOTION_PROF_kP));
        applyToAllAngle((angleMotor) -> angleMotor.config_kI(ANGLE_MOTION_PROF_SLOT, ANGLE_MOTION_PROF_kI));
        applyToAllAngle((angleMotor) -> angleMotor.config_kD(ANGLE_MOTION_PROF_SLOT, ANGLE_MOTION_PROF_kD));
        
        applyToAllDrive((talon) -> talon.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, MOTION_FRAME_PERIOD));
        applyToAllAngle((talon) -> talon.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, MOTION_FRAME_PERIOD));
    }

    /**
     * Sets the output of the drivetrain based on desired output vectors for each swerve module
     */
    public void setDrivetrain(Vector tl, Vector tr, Vector bl, Vector br, boolean isPercentOutput) {
        double tlOutput = isPercentOutput ? tl.getMagnitude() : tl.getMagnitude() * MAX_DRIVE_VELOCITY;
        double trOutput = isPercentOutput ? tr.getMagnitude() : tr.getMagnitude() * MAX_DRIVE_VELOCITY;
        double blOutput = isPercentOutput ? bl.getMagnitude() : bl.getMagnitude() * MAX_DRIVE_VELOCITY;
        double brOutput = isPercentOutput ? br.getMagnitude() : br.getMagnitude() * MAX_DRIVE_VELOCITY;
        
        setSwerveModule(topLeft, tlOutput, convertAngle(topLeft, tl.getAngle()), isPercentOutput);
		setSwerveModule(topRight, trOutput, convertAngle(topRight, tr.getAngle()), isPercentOutput);
		setSwerveModule(backLeft, blOutput, convertAngle(backLeft, bl.getAngle()), isPercentOutput);
        setSwerveModule(backRight, brOutput, convertAngle(backRight, br.getAngle()), isPercentOutput);
    }

    public void setSwerveModule(SwerveModule module, double output, double angle, boolean isPercentOutput) {
        module.setAngleAndDrive(angle, output, isPercentOutput);
    }

    /** 
     * Converts the target angle from the desired Vectors into the actual angle for the motors to hold.
     * Angle modification for Field sensitive drive should have already been handled.
     * 
     * Steps:
     * 
     * 1. Subtract 90 degrees. 0 degrees on the Joysticks and desired Vectors points to the right (positive x axis)
     *    while 0 degrees on the robot points forward (positive y axis). The subtraction deals with this offset.
     * 2. Increase/Decrease the targetAngle by 360 degrees until it is within +- 180 degrees of the current angle
     * 
     * @return The desired angle after all modifications
     */
    public static double convertAngle(SwerveModule module, double targetAngle) {
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
        
        return targetAngle;
    }

    /**
     * Stops all drive motors while holding the current angle
     */
    public void stopAllDrive() {
        setSwerveModule(topLeft, 0, topLeft.getAngleDegrees(), true);
        setSwerveModule(topRight, 0, topRight.getAngleDegrees(), true);
        setSwerveModule(backLeft, 0, backLeft.getAngleDegrees(), true);
        setSwerveModule(backRight, 0, backRight.getAngleDegrees(), true);
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

    public boolean isFieldSensitive() {
        return isFieldSensitive;
    }

    public static Drivetrain getInstance() {
        if(instance == null) 
            instance = new Drivetrain();
        return instance;
    }
}
