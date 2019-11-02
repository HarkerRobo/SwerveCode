package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import harkerrobolib.wrappers.HSTalon;

public class SwerveModule {

    private static final double VOLTAGE_COMP = 10;

    private static final int CURRENT_CONTINUOUS = 20;
    private static final int CURRENT_PEAK = 30;
    private static final int CURRENT_PEAK_DUR = 500;

    private HSTalon angleMotor;
    private HSTalon driveMotor;

    private static final boolean TOP_LEFT_INVERTED = false;
    private static final boolean TOP_RIGHT_INVERTED = false;
    private static final boolean BOTTOM_LEFT_INVERTED = false;
    private static final boolean BOTTOM_RIGHT_INVERTED = false;

    public SwerveModule(int driveId, int angleId) {
        driveMotor = new HSTalon(driveId);
        angleMotor = new HSTalon(angleId);

        talonInit(driveMotor);
        talonInit(angleMotor);
    }        
    
    public HSTalon getAngleMotor() {
        return angleMotor;    
    }            
        
    public HSTalon getDriveMotor() {
        return driveMotor;
    } 
    
    public static void talonInit(HSTalon talon) {
        talon.configFactoryDefault();
        
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        talon.setNeutralMode(NeutralMode.Brake);

        talon.configVoltageCompSaturation(VOLTAGE_COMP);
        talon.enableVoltageCompensation(true);

        configCurrentLimit(talon);
    }

    public static void configCurrentLimit(HSTalon talon) {
        talon.configContinuousCurrentLimit(CURRENT_CONTINUOUS);
        talon.configPeakCurrentLimit(CURRENT_PEAK);
        talon.configPeakCurrentDuration(CURRENT_PEAK_DUR);
        talon.enableCurrentLimit(true);
    }
}