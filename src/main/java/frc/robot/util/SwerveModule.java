package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import harkerrobolib.wrappers.HSTalon;

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

    //Voltage/Current Constants
    private static final double VOLTAGE_COMP = 10;

    private static final int CURRENT_CONTINUOUS = 40;
    private static final int CURRENT_PEAK = 60;
    private static final int CURRENT_PEAK_DUR = 500;

    // Motor inversions
    private final boolean DRIVE_INVERTED;
    private final boolean ANGLE_INVERTED;

    public boolean swerveDriveInverted; // Whether the motor is inverted when turning the angle motors

    private final boolean DRIVE_SENSOR_PHASE;
    private final boolean ANGLE_SENSOR_PHASE;

    private HSTalon angleMotor;
    private HSTalon driveMotor;
    
    private Vector velocity;

    public SwerveModule(int driveId, boolean invertDriveTalon, boolean driveSensorPhase, int angleId, boolean invertAngleTalon, boolean angleSensorPhase) {
        driveMotor = new HSTalon(driveId);
        angleMotor = new HSTalon(angleId);

        DRIVE_INVERTED = invertDriveTalon;
        ANGLE_INVERTED = invertAngleTalon;

        DRIVE_SENSOR_PHASE = driveSensorPhase;
        ANGLE_SENSOR_PHASE = angleSensorPhase;

        swerveDriveInverted = false;

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

    public boolean shouldInvert(int desiredPos) {
        return Math.abs(talon.getAngleMotor().getSelectedSensorPosition() - desiredPos) > 90;
    }

    /**
     * Sets the target angle of the swerve module.
     * 
     * @param targetAngle the angle (in degrees) of the setpoint
     */
    public void setTargetAngle(double targetAngle) {
        targetAngle = targetAngle % 360;
        targetAngle += mZeroOffset;
        double currentAngle = angleMotor.getSelectedSensorPosition(0) * (360.0/1024.0);
        double currentAngleMod = modulate360(currentAngle);
        if (currentAngleMod < 0) currentAngleMod += 360;
        
        double delta = currentAngleMod - targetAngle;
        while (delta > 180) {
            targetAngle += 360;
        }
        while (delta < -180) {
            targetAngle -= 360;
        }
       
        targetAngle += currentAngle - currentAngleMod;
        lastTargetAngle = targetAngle;
        
        int rawAngle = convertDegreesToEncoder(targetAngle);
        setRawAngle(rawAngle);
    }

    public void setRawAngle(int rawAngle)  {
        angleMotor.set(rawAngle);
    }

    private int toCounts(double angle)
    {
        return (angle*RobotMap.ENCODER_TICKS_PER_REVOLUTION)/360.0;
    }
    
    public void setTargetSpeed(double speed) {
        
    }

    public double getAngle() {
        return velocity.getAngle();
    }
}