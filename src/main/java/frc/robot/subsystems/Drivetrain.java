package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;
import harkerrobolib.util.Conversions;
import harkerrobolib.wrappers.HSPigeon;
import harkerrobolib.wrappers.HSTalon;

import java.util.function.Consumer;

import com.ctre.phoenix.sensors.PigeonIMUConfiguration;

/**
 * Simulates the drivetrain subsystem on the robot. A Swerve Drivetrain contains
 * four SwerveModules.
 * 
 * 'back' is defined as closest to the battery 'left' is defined as left when
 * standing at the back and looking forward
 * 
 * Acronyms: TL: Top Left TR: Top Right BL: Back Left BR: Back Right
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Arjun Dixit
 * @since 11/1/19
 */
public class Drivetrain extends SubsystemBase {
    public static Drivetrain instance;

    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    // private boolean isFieldSensitive;
    private HSPigeon pigeon;
    private SwerveDriveOdometry odometry;
    private SwerveDriveKinematics kinematics;

    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(-Drivetrain.DT_WIDTH/2, Drivetrain.DT_LENGTH/2);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(Drivetrain.DT_WIDTH/2, Drivetrain.DT_LENGTH/2);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-Drivetrain.DT_WIDTH/2, -Drivetrain.DT_LENGTH/2);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(Drivetrain.DT_WIDTH/2, -Drivetrain.DT_LENGTH/2);

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
    private static final double ANGLE_POSITION_KP = 0.85;
    private static final double ANGLE_POSITION_KI = 0.0;
    private static final double ANGLE_POSITION_KD = 30.0;

    public static final int DRIVE_VELOCITY_SLOT = 0;
    private static final double DRIVE_VELOCITY_KP = 0.13;
    private static final double DRIVE_VELOCITY_KI = 0.0;
    private static final double DRIVE_VELOCITY_KD = 0.0;
    private static final double DRIVE_VELOCITY_KF = 0.046; // theoretical: 0.034;

    public static final double MAX_DRIVE_VELOCITY = 2;
    public static final double MAX_ROTATION_VELOCITY = (2 * Math.PI);
    public static final double MAX_ROTATION_ACCELERATION = 2 * (2 * Math.PI);
    public static final double MAX_DRIVE_ACCELERATION = 1.2;

    public static final double DRIVE_RAMP_RATE = 0.1;
    public static final double ANGLE_RAMP_RATE = 0.05;
    public static final double GEAR_RATIO = 6;

    public static final double METERS_PER_FOOT = 0.3048;
    public static final double FEET_PER_METER = 3.28084;

    /**
     * Feet between both of the wheels on the front or back
     */
    public static final double DT_WIDTH = 0.419; //16.5 feet;
    /**
     * Feet between both of the wheels on the left or right
     */
    public static final double DT_LENGTH = 0.523; //20.6 feet;

    public static final int TL_OFFSET = 2212;
    public static final int TR_OFFSET = 6730;
    public static final int BL_OFFSET = 11327;
    private static final int BR_OFFSET = 4605;

    public static final double PIGEON_kP = 0.10;

    public static final double WHEEL_DIAMETER = 4;

    public static final double MP_X_KP = 2.6;
    public static final double MP_X_KI = 0;
    public static final double MP_X_KD = 15;

    public static final double MP_Y_KP = 0.7;
    public static final double MP_Y_KI = 0;
    public static final double MP_Y_KD = 0;

    public static final double MP_THETA_KP = 3.1;
    public static final double MP_THETA_KI = 0;
    public static final double MP_THETA_KD = 0;
	public static final Constraints THETA_CONSTRAINTS = new Constraints(MAX_ROTATION_VELOCITY, MAX_ROTATION_ACCELERATION);

    private Drivetrain() {
        topLeft = new SwerveModule(RobotMap.TL_DRIVE_ID, TL_DRIVE_INVERTED, TL_DRIVE_SENSOR_PHASE, RobotMap.TL_ANGLE_ID,
                TL_ANGLE_INVERTED, TL_ANGLE_SENSOR_PHASE);
        topRight = new SwerveModule(RobotMap.TR_DRIVE_ID, TR_DRIVE_INVERTED, TR_DRIVE_SENSOR_PHASE,
                RobotMap.TR_ANGLE_ID, TR_ANGLE_INVERTED, TR_ANGLE_SENSOR_PHASE);
        backLeft = new SwerveModule(RobotMap.BL_DRIVE_ID, BL_DRIVE_INVERTED, BL_DRIVE_SENSOR_PHASE,
                RobotMap.BL_ANGLE_ID, BL_ANGLE_INVERTED, BL_ANGLE_SENSOR_PHASE);
        backRight = new SwerveModule(RobotMap.BR_DRIVE_ID, BR_DRIVE_INVERTED, BR_DRIVE_SENSOR_PHASE,
                RobotMap.BR_ANGLE_ID, BR_ANGLE_INVERTED, BR_ANGLE_SENSOR_PHASE);

        int tlAngleOffset = (topLeft.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - TL_OFFSET) / 4;
        int trAngleOffset = (topRight.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - TR_OFFSET) / 4;
        int blAngleOffset = (backLeft.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - BL_OFFSET) / 4;
        int brAngleOffset = (backRight.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - BR_OFFSET) / 4;

        topLeft.getAngleMotor().setSelectedSensorPosition(tlAngleOffset);
        topRight.getAngleMotor().setSelectedSensorPosition(trAngleOffset);
        backLeft.getAngleMotor().setSelectedSensorPosition(blAngleOffset);
        backRight.getAngleMotor().setSelectedSensorPosition(brAngleOffset);

        applyToAllDrive((motor) -> motor.setSelectedSensorPosition(0));

        setupPositionPID();
        setupVelocityPID();

        // isFieldSensitive = true;
        pigeon = new HSPigeon(RobotMap.PIGEON_ID);
        pigeon.configFactoryDefault();
        pigeon.zero();        
        pigeon.setYaw(90);
        System.out.println(pigeon.setFusedHeading(90));
        System.out.println(pigeon.setAccumZAngle(90));
        Conversions.setWheelDiameter(WHEEL_DIAMETER);

        kinematics = new SwerveDriveKinematics(Drivetrain.FRONT_LEFT_LOCATION, Drivetrain.FRONT_RIGHT_LOCATION,
                Drivetrain.BACK_LEFT_LOCATION, Drivetrain.BACK_RIGHT_LOCATION);

        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(pigeon.getFusedHeading() +90), new Pose2d(new Translation2d(), Rotation2d.fromDegrees(/**90 */ 0 )));

    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(pigeon.getFusedHeading() + 90),
                Drivetrain.getInstance().getTopLeft().getState(), Drivetrain.getInstance().getTopRight().getState(),
                Drivetrain.getInstance().getBackLeft().getState(), Drivetrain.getInstance().getBackRight().getState());

        SmartDashboard.putNumber("Current X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Current Y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("Current Rot", getPose().getRotation().getDegrees());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        Pose2d rawPose = odometry.getPoseMeters();

        return new Pose2d(new Translation2d(-rawPose.getTranslation().getY(), rawPose.getTranslation().getX()), 
                rawPose.getRotation());
    }

    // public void toggleFieldSensitivity() {
    //     isFieldSensitive = !isFieldSensitive;
    //     System.out.println(isFieldSensitive);
    // }

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

    public void setDrivetrainModuleStates(SwerveModuleState[] states) {
        setDrivetrainVelocity(states[0], states[1], states[2], states[3], 0, false, true);
    }

    /**
     * Sets the output of the drivetrain based on desired output vectors for each
     * swerve module
     */
    public void setDrivetrainVelocity(SwerveModuleState tl, SwerveModuleState tr, SwerveModuleState bl,
            SwerveModuleState br, double feedForward, boolean isPercentOutput, boolean isMotionProfile) {
        double tlOutput = tl.speedMetersPerSecond;
        double trOutput = tr.speedMetersPerSecond;
        double blOutput = bl.speedMetersPerSecond;
        double brOutput = br.speedMetersPerSecond;

        setSwerveModuleVelocity(topLeft, tlOutput, convertAngle(topLeft, tl.angle.getDegrees()), isPercentOutput,
                isMotionProfile);
        setSwerveModuleVelocity(topRight, trOutput, convertAngle(topRight, tr.angle.getDegrees()), isPercentOutput,
                isMotionProfile);
        setSwerveModuleVelocity(backLeft, blOutput, convertAngle(backLeft, bl.angle.getDegrees()), isPercentOutput,
                isMotionProfile);
        setSwerveModuleVelocity(backRight, brOutput, convertAngle(backRight, br.angle.getDegrees()), isPercentOutput,
                isMotionProfile);
    }

    public void setSwerveModuleVelocity(SwerveModule module, double output, double angle, boolean isPercentOutput,
            boolean isMotionProfile) {
        module.setAngleAndDriveVelocity(angle, output, isPercentOutput, isMotionProfile);
    }

    /**
     * Converts the target angle from the desired Vectors into the actual angle for
     * the motors to hold. Angle modification for Field sensitive drive should have
     * already been handled.
     * 
     * Steps:
     * 
     * 1. Subtract 90 degrees. 0 degrees on the Joysticks and desired Vectors points
     * to the right (positive x axis) while 0 degrees on the robot points forward
     * (positive y axis). The subtraction deals with this offset. 2.
     * Increase/Decrease the targetAngle by 360 degrees until it is within +- 180
     * degrees of the current angle
     * 
     * @return The desired angle after all modifications
     */
    public static double convertAngle(SwerveModule module, double targetAngle) {
        // Step 1
        targetAngle -= 90;

        double currDegrees = module.getAngleDegrees();

        // Step 2
        while (currDegrees - targetAngle > 180) {
            targetAngle += 360;
        }
        while (currDegrees - targetAngle < -180) {
            targetAngle -= 360;
        }

        return targetAngle;
    }

    
    
    public void setPigeonToNinety() {
        pigeon.setFusedHeading(90);
    }

    // public boolean isFieldSensitive() {
    //     return isFieldSensitive;
    // }

    /**
     * Stops all drive motors while holding the current angle
     */
    public void stopAllDrive() {
        setSwerveModuleVelocity(topLeft, 0, topLeft.getAngleDegrees(), true, false);
        setSwerveModuleVelocity(topRight, 0, topRight.getAngleDegrees(), true, false);
        setSwerveModuleVelocity(backLeft, 0, backLeft.getAngleDegrees(), true, false);
        setSwerveModuleVelocity(backRight, 0, backRight.getAngleDegrees(), true, false);
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

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }
}
