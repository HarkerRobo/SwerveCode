package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import harkerrobolib.wrappers.HSTalon;

/**
 * Simulates a swerve module on the drivetrain.
 * 
 * @version 11/3/19
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 */
public class SwerveModule {

    private static final double VOLTAGE_COMP = 10;

    private static final int CURRENT_CONTINUOUS = 20;
    private static final int CURRENT_PEAK = 30;
    private static final int CURRENT_PEAK_DUR = 500;

    private HSTalon angleMotor;
    private HSTalon driveMotor;

    private final boolean DRIVE_INVERTED;
    private final boolean ANGLE_INVERTED;

    private final boolean DRIVE_SENSOR_PHASE;
    private final boolean ANGLE_SENSOR_PHASE;

    public SwerveModule(int driveId, boolean invertDriveTalon, boolean driveSensorPhase, int angleId, boolean invertAngleTalon, boolean angleSensorPhase) {
        driveMotor = new HSTalon(driveId);
        angleMotor = new HSTalon(angleId);

        DRIVE_INVERTED = invertDriveTalon;
        ANGLE_INVERTED = invertAngleTalon;

        DRIVE_SENSOR_PHASE = driveSensorPhase;
        ANGLE_SENSOR_PHASE = angleSensorPhase;

        driveTalonInit(driveMotor, DRIVE_INVERTED, DRIVE_SENSOR_PHASE);
        angleTalonInit(angleMotor, ANGLE_INVERTED, ANGLE_SENSOR_PHASE);
    }        
    
    public HSTalon getAngleMotor() {
        return angleMotor;    
    }            
        
    public HSTalon getDriveMotor() {
        return driveMotor;
    } 
    
    public static void driveTalonInit(HSTalon talon, boolean invert, boolean sensorPhase) {
        talon.configFactoryDefault();
        
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        talon.setNeutralMode(NeutralMode.Brake);

        talon.configVoltageCompSaturation(VOLTAGE_COMP);
        talon.enableVoltageCompensation(true);

        talon.setInverted(invert);
        talon.setSensorPhase(sensorPhase);

        configCurrentLimit(talon);
    }

    public static void angleTalonInit(HSTalon talon, boolean invert, boolean sensorPhase) {
        talon.configFactoryDefault();
        
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        talon.setNeutralMode(NeutralMode.Brake);

        talon.configVoltageCompSaturation(VOLTAGE_COMP);
        talon.enableVoltageCompensation(true);

        talon.setInverted(invert);
        talon.setSensorPhase(sensorPhase);

        configCurrentLimit(talon);
    }

    public static void configCurrentLimit(HSTalon talon) {
        talon.configContinuousCurrentLimit(CURRENT_CONTINUOUS);
        talon.configPeakCurrentLimit(CURRENT_PEAK);
        talon.configPeakCurrentDuration(CURRENT_PEAK_DUR);
        talon.enableCurrentLimit(true);
    }
}