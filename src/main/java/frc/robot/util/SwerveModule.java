package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;
import harkerrobolib.wrappers.HSTalon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

/**
 * A swerve module on the drivetrain.
 * A swerve module contains one motor to spin the wheel and another to change the wheel's angle.
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Arjun Dixit
 * @since 11/1/19
 */
public class SwerveModule {

    //Encoder ticks per one rotation of the wheel angle gear, gear ratio is 1:1
    public static final int ENCODER_TICKS = 4096;

    //Voltage/Current Constants
    private static final double VOLTAGE_COMP = 10;

    private static final int DRIVE_CURRENT_CONTINUOUS = 40;
    private static final int DRIVE_CURRENT_PEAK = 60;
    private static final int ANGLE_CURRENT_CONTINUOUS = 15;
    private static final int ANGLE_CURRENT_PEAK = 15;
    private static final int CURRENT_PEAK_DUR = 50;

    // Motor inversions
    private final boolean DRIVE_INVERTED;
    private final boolean ANGLE_INVERTED;

    private final boolean DRIVE_SENSOR_PHASE;
    private final boolean ANGLE_SENSOR_PHASE;

    // Whether the drive motor should be inverted due to turning logic
    private boolean swerveDriveInverted; 
    private boolean invertFlag;
    private HSTalon angleMotor;
    private HSTalon driveMotor;

    public SwerveModule(int driveId, boolean invertDriveTalon, boolean driveSensorPhase, int angleId, boolean invertAngleTalon, boolean angleSensorPhase) {
        swerveDriveInverted = false;
        
        driveMotor = new HSTalon(driveId);
        angleMotor = new HSTalon(angleId);

        DRIVE_INVERTED = invertDriveTalon;
        ANGLE_INVERTED = invertAngleTalon;

        DRIVE_SENSOR_PHASE = driveSensorPhase;
        ANGLE_SENSOR_PHASE = angleSensorPhase;

        driveTalonInit(driveMotor);
        angleTalonInit(angleMotor);

        invertFlag = false;
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

        talon.configContinuousCurrentLimit(DRIVE_CURRENT_CONTINUOUS);
        talon.configPeakCurrentLimit(DRIVE_CURRENT_PEAK);
        talon.configPeakCurrentDuration(CURRENT_PEAK_DUR);
        talon.enableCurrentLimit(true);

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

        talon.configContinuousCurrentLimit(ANGLE_CURRENT_CONTINUOUS);
        talon.configPeakCurrentLimit(ANGLE_CURRENT_PEAK);
        talon.configPeakCurrentDuration(CURRENT_PEAK_DUR);
        talon.enableCurrentLimit(true);

        talon.configVoltageCompSaturation(VOLTAGE_COMP);
        talon.enableVoltageCompensation(true);
    }

    public void invertOutput() {
        swerveDriveInverted = !swerveDriveInverted;
    }
 
    /**
     * Sets the drive output of the swerve module in either percent output or velocity in feet per second.
     * 
     * @param output the output of the swerve module
     * @param isPercentOutput true if the output is in percent output, false if it is in feet per second.
     */
    public void setDriveOutput(double output, boolean isPercentOutput) {
        if(isPercentOutput) {
            driveMotor.set(ControlMode.PercentOutput, output);
        } else {
            driveMotor.set(ControlMode.Velocity, Conversions.convert(SpeedUnit.FEET_PER_SECOND, output * Drivetrain.FEET_PER_METER, SpeedUnit.ENCODER_UNITS) * Drivetrain.GEAR_RATIO);
        }
    }
    
    public void setAngleAndDriveVelocity(double targetAngle, double output, boolean isPercentOutput, boolean isMotionProfile) {
        boolean shouldReverse = !isMotionProfile && Math.abs(targetAngle - getAngleDegrees()) > 90;
        
        if (shouldReverse) {
            setDriveOutput(-output, isPercentOutput);
            if (targetAngle - getAngleDegrees() > 90) {
                targetAngle -= 180;
            }
            else {
                targetAngle += 180;
            }
        } else {
            setDriveOutput(output, isPercentOutput);
        }
        
        int targetPos = (int)((targetAngle / 360) * 4096);

        angleMotor.set(ControlMode.Position, targetPos);
    }

    public void setAngleAndDrivePosition(double targetAngle, double position, double feedForward) {
        boolean shouldReverse = Math.abs(targetAngle - getAngleDegrees()) > 90;
        
        if (shouldReverse) {
            position = driveMotor.getSelectedSensorPosition() - Math.signum(position) * (position - driveMotor.getSelectedSensorPosition());
            if (targetAngle - getAngleDegrees() > 90) {
                targetAngle -= 180;
            }
            else {
                targetAngle += 180;
            }
        }
        
        int targetPos = (int)((targetAngle / 360) * 4096);

        angleMotor.set(ControlMode.Position, targetPos);
        driveMotor.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, feedForward);
    }

    /**
     * Returns the current angle in degrees
     */
    public double getAngleDegrees() {
        return angleMotor.getSelectedSensorPosition() * 360.0 / SwerveModule.ENCODER_TICKS; //Convert encoder ticks to degrees
    }

    public HSTalon getAngleMotor() {
        return angleMotor;    
    }            
        
    public HSTalon getDriveMotor() {
        return driveMotor;
    }

     /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(Conversions.convertSpeed(SpeedUnit.ENCODER_UNITS, driveMotor.getSelectedSensorVelocity(), SpeedUnit.FEET_PER_SECOND) * Drivetrain.METERS_PER_FOOT, 
            new Rotation2d(angleMotor.getSelectedSensorPosition() * 2 * Math.PI / 4096));
    }
}