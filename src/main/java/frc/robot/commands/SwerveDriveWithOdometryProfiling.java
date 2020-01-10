package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveDriveWithOdometryProfiling extends CommandBase {

    // Creating my kinematics object using the module locations
    
    private SwerveDriveOdometry odometry;
    

    public SwerveDriveWithOdometryProfiling(SwerveDriveOdometry odometry) {
        addRequirements(Drivetrain.getInstance());
        this.odometry = odometry;
    }

    public void initialize() {
        Rotation2d gyroAngle = Rotation2d.fromDegrees(-Drivetrain.getInstance().getPigeon().getFusedHeading());
        odometry.resetPosition(new Pose2d(, 0, new Rotation2d()), gyroAngle);
    }
        
    public void execute() {
        Rotation2d gyroAngle = Rotation2d.fromDegrees(-Drivetrain.getInstance().getPigeon().getFusedHeading());
        //Pose2d odometry.getPoseMeters();
        Pose2d newPose = odometry.update(gyroAngle, Drivetrain.getInstance().getTopLeft().getState(), 
                                                   Drivetrain.getInstance().getTopRight().getState(),
                                                   Drivetrain.getInstance().getBackLeft().getState(), 
                                                   Drivetrain.getInstance().getBackRight().getState());
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted)
    {
        
    }
    
}