package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveDriveWithOdometryProfiling extends CommandBase {

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    Drivetrain.FRONT_LEFT_LOCATION, 
    Drivetrain.FRONT_RIGHT_LOCATION, 
    Drivetrain.BACK_LEFT_LOCATION, 
    Drivetrain.BACK_RIGHT_LOCATION
    );
    
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
        new Rotation2d(Drivetrain.getInstance().getPigeon().getFusedHeading() * Math.PI / 180), new Pose2d(5.0, 13.5, new Rotation2d()));

    public SwerveDriveWithOdometryProfiling() {
        addRequirements(Drivetrain.getInstance());
    }
        
    public void periodic() {
        Rotation2d gyroAngle = Rotation2d.fromDegrees(-Drivetrain.getInstance().getPigeon().getFusedHeading());
        Pose2d pose = m_odometry.update(gyroAngle, Drivetrain.getInstance().getTopLeft().getState(), 
                                                   Drivetrain.getInstance().getTopRight().getState(),
                                                   Drivetrain.getInstance().getBackLeft().getState(), 
                                                   Drivetrain.getInstance().getBackRight().getState());

    }

    public void end(boolean interrupted)
    {
        
    }
    
}