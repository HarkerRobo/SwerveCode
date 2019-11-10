package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

/**
 * Runs each swerve module with the same output.
 * On the driver controller:
 *      A moves the drive motors forward (spins wheels)
 *      B moves the drive motors backward (spins wheels in opposite direction)
 *      Y moves the angle motors forward (rotates wheel angles)
 *      X moves the angle motors backward (rotates wheel angles in opposite direction)
 * 
 * @author Chirag Kaushik
 * @since 11/4/19
 */
public class SwerveTest extends Command {
    private static final double OUTPUT = 0.2;

    public SwerveTest() {
        requires(Drivetrain.getInstance());
    }

    public void execute() {
        if(OI.getInstance().getDriverGamepad().getButtonAState()) {
            Drivetrain.getInstance().getTopRight().getDriveMotor().set(ControlMode.PercentOutput, OUTPUT);
            Drivetrain.getInstance().getBackRight().getDriveMotor().set(ControlMode.PercentOutput, OUTPUT);
            Drivetrain.getInstance().getTopLeft().getDriveMotor().set(ControlMode.PercentOutput, OUTPUT);
            Drivetrain.getInstance().getBackLeft().getDriveMotor().set(ControlMode.PercentOutput, OUTPUT);
        } 
        else if(OI.getInstance().getDriverGamepad().getButtonBState()) {
            Drivetrain.getInstance().getTopRight().getDriveMotor().set(ControlMode.PercentOutput, -OUTPUT);
            Drivetrain.getInstance().getBackRight().getDriveMotor().set(ControlMode.PercentOutput, -OUTPUT);
            Drivetrain.getInstance().getTopLeft().getDriveMotor().set(ControlMode.PercentOutput, -OUTPUT);
            Drivetrain.getInstance().getBackLeft().getDriveMotor().set(ControlMode.PercentOutput, -OUTPUT);
        } else {
            Drivetrain.getInstance().getTopRight().getDriveMotor().set(ControlMode.Disabled, 0);
            Drivetrain.getInstance().getBackRight().getDriveMotor().set(ControlMode.Disabled, 0);
            Drivetrain.getInstance().getTopLeft().getDriveMotor().set(ControlMode.Disabled, 0);
            Drivetrain.getInstance().getBackLeft().getDriveMotor().set(ControlMode.Disabled, 0);
        }
        if(OI.getInstance().getDriverGamepad().getButtonYState()) {
            Drivetrain.getInstance().getTopRight().getAngleMotor().set(ControlMode.PercentOutput, OUTPUT);
            Drivetrain.getInstance().getBackRight().getAngleMotor().set(ControlMode.PercentOutput, OUTPUT);
            Drivetrain.getInstance().getTopLeft().getAngleMotor().set(ControlMode.PercentOutput, OUTPUT);
            Drivetrain.getInstance().getBackLeft().getAngleMotor().set(ControlMode.PercentOutput, OUTPUT);
        } else 
        if(OI.getInstance().getDriverGamepad().getButtonXState()) {
            Drivetrain.getInstance().getTopRight().getAngleMotor().set(ControlMode.PercentOutput, -OUTPUT);
            Drivetrain.getInstance().getBackRight().getAngleMotor().set(ControlMode.PercentOutput,- OUTPUT);
            Drivetrain.getInstance().getTopLeft().getAngleMotor().set(ControlMode.PercentOutput, -OUTPUT);
            Drivetrain.getInstance().getBackLeft().getAngleMotor().set(ControlMode.PercentOutput, -OUTPUT);
        } else {
            Drivetrain.getInstance().getTopRight().getAngleMotor().set(ControlMode.Disabled, 0);
            Drivetrain.getInstance().getBackRight().getAngleMotor().set(ControlMode.Disabled, 0);
            Drivetrain.getInstance().getTopLeft().getAngleMotor().set(ControlMode.Disabled, 0);
            Drivetrain.getInstance().getBackLeft().getAngleMotor().set(ControlMode.Disabled, 0);
        }

        SmartDashboard.putNumber("dt tr angle", Drivetrain.getInstance().getTopRight().getAngleMotor().getSelectedSensorPosition());
        SmartDashboard.putNumber("dt tl angle", Drivetrain.getInstance().getTopLeft().getAngleMotor().getSelectedSensorPosition());
        SmartDashboard.putNumber("dt br angle", Drivetrain.getInstance().getBackRight().getAngleMotor().getSelectedSensorPosition());
        SmartDashboard.putNumber("dt bl angle", Drivetrain.getInstance().getBackLeft().getAngleMotor().getSelectedSensorPosition());

        SmartDashboard.putNumber("dt tr drive", Drivetrain.getInstance().getTopRight().getDriveMotor().getSelectedSensorPosition());
        SmartDashboard.putNumber("dt tl drive", Drivetrain.getInstance().getTopLeft().getDriveMotor().getSelectedSensorPosition());
        SmartDashboard.putNumber("dt br drive", Drivetrain.getInstance().getBackRight().getDriveMotor().getSelectedSensorPosition());
        SmartDashboard.putNumber("dt bl drive", Drivetrain.getInstance().getBackLeft().getDriveMotor().getSelectedSensorPosition());
    }

    public void end() {

        Drivetrain.getInstance().getTopRight().getDriveMotor().set(ControlMode.Disabled, 0);
        Drivetrain.getInstance().getBackRight().getDriveMotor().set(ControlMode.Disabled, 0);
        Drivetrain.getInstance().getTopLeft().getDriveMotor().set(ControlMode.Disabled, 0);
        Drivetrain.getInstance().getBackLeft().getDriveMotor().set(ControlMode.Disabled, 0);

        Drivetrain.getInstance().getTopRight().getAngleMotor().set(ControlMode.Disabled, 0);
        Drivetrain.getInstance().getBackRight().getAngleMotor().set(ControlMode.Disabled, 0);
        Drivetrain.getInstance().getTopLeft().getAngleMotor().set(ControlMode.Disabled, 0);
        Drivetrain.getInstance().getBackLeft().getAngleMotor().set(ControlMode.Disabled, 0);
    }


    public void interrupted() {
        end();
    }

    public boolean isFinished() {
        return false;
    }
}