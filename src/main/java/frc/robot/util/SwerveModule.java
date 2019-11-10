package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.RobotMap;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.AngleUnit;
import harkerrobolib.wrappers.HSTalon;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * A swerve module on the drivetrain.
 * A swerve module contains one motor to spin the wheel and another to change the wheel's angle.
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @since 11/1/19
 */
public class SwerveModule {

    public static final int ENCODER_TICKS = 4096;


    //Voltage/Current Constants
    private static final double VOLTAGE_COMP = 10;

    private static final int CURRENT_CONTINUOUS = 40;
    private static final int CURRENT_PEAK = 60;
    private static final int CURRENT_PEAK_DUR = 500;

    // Motor inversions
    private final boolean DRIVE_INVERTED;
    private final boolean ANGLE_INVERTED;
    public boolean swerveDriveInverted; // Whether the drive motor is inverted when turning the angle motors

    private final boolean DRIVE_SENSOR_PHASE;
    private final boolean ANGLE_SENSOR_PHASE;

    private HSTalon angleMotor;
    private HSTalon driveMotor;
    
    // private Vector velocity;

    public SwerveModule(int driveId, boolean invertDriveTalon, boolean driveSensorPhase, int angleId, boolean invertAngleTalon, boolean angleSensorPhase) {
        driveMotor = new HSTalon(driveId);
        angleMotor = new HSTalon(angleId);

        DRIVE_INVERTED = invertDriveTalon;
        ANGLE_INVERTED = invertAngleTalon;

        DRIVE_SENSOR_PHASE = driveSensorPhase;
        ANGLE_SENSOR_PHASE = angleSensorPhase;

        // swerveDriveInverted = false;

        driveTalonInit(driveMotor);
        angleTalonInit(angleMotor);
    }
    
    public void driveTalonInit(HSTalon talon) {
        talon.configFactoryDefault();
        
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        talon.setNeutralMode(NeutralMode.Brake);

        talon.setInverted(DRIVE_INVERTED);
        talon.setSensorPhase(DRIVE_SENSOR_PHASE);
        
        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);
        talon.overrideLimitSwitchesEnable(false); 

        talon.setSelectedSensorPosition(0);

        configCurrentLimit(talon);

        talon.configVoltageCompSaturation(VOLTAGE_COMP);
        talon.enableVoltageCompensation(true);
        
    }

    public void angleTalonInit(HSTalon talon) {
        talon.configFactoryDefault();
        
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        talon.setNeutralMode(NeutralMode.Brake);

        talon.setInverted(ANGLE_INVERTED);
        talon.setSensorPhase(ANGLE_SENSOR_PHASE);

        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);
        talon.overrideLimitSwitchesEnable(false);

        configCurrentLimit(talon);

        talon.configVoltageCompSaturation(VOLTAGE_COMP);
        talon.enableVoltageCompensation(true);
    } 
    
    public static void configCurrentLimit(HSTalon talon) {
        talon.configContinuousCurrentLimit(CURRENT_CONTINUOUS);
        talon.configPeakCurrentLimit(CURRENT_PEAK);
        talon.configPeakCurrentDuration(CURRENT_PEAK_DUR);
        talon.enableCurrentLimit(true);
    }
    
    public HSTalon getAngleMotor() {
        return angleMotor;    
    }            
        
    public HSTalon getDriveMotor() {
        return driveMotor;
    }

    public void setOutput(double output) {
        driveMotor.set(ControlMode.PercentOutput, output * (swerveDriveInverted ? -1 : 1));
    }

    public void invertOutput() {
        swerveDriveInverted = !swerveDriveInverted;
    }

    /**
     * Sets the target angle of the swerve module.
     * 
     * @param targetAngle the angle (in degrees) of the setpoint
     */
    public void setTargetAngle(double targetAngle) { 
        int rawAngle = (int)((targetAngle / 360) * 4096);
        angleMotor.set(ControlMode.Position, rawAngle);
        System.out.println(rawAngle);
    }

    /**
     * Returns the current angle in degrees from [0, 360)
     */
    public double getAngleDegrees() {
        double rawAngle = angleMotor.getSelectedSensorPosition() * 360.0 / SwerveModule.ENCODER_TICKS; //Convert encoder ticks to degrees
        return rawAngle;// % 360 + 360) % 360; //Map Degrees into the range [0, 360) and return
    }

}