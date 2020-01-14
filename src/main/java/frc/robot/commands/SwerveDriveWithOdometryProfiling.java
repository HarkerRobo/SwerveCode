package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Controls the drivetrain using WPILib's SwerveControllerCommand with 
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Arjun Dixit
 */
public class SwerveDriveWithOdometryProfiling extends SwerveControllerCommand {
    
    private Timer timer;
    private Trajectory trajectory;
        
    public SwerveDriveWithOdometryProfiling(Trajectory trajectory) {
        super(trajectory,
            Drivetrain.getInstance()::getPose,
            Drivetrain.getInstance().getKinematics(), 
            new PIDController(Drivetrain.MP_X_KP, Drivetrain.MP_X_KI, Drivetrain.MP_X_KD), 
            new PIDController(Drivetrain.MP_Y_KP, Drivetrain.MP_Y_KI, Drivetrain.MP_Y_KD), 
            new ProfiledPIDController(Drivetrain.MP_THETA_KP, Drivetrain.MP_THETA_KI, Drivetrain.MP_THETA_KD,
                                    Drivetrain.THETA_CONSTRAINTS),
            Drivetrain.getInstance()::setDrivetrainModuleStates,
            Drivetrain.getInstance()
        );
        
        this.trajectory = trajectory;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        super.initialize();

        timer.start();
        
        // Drivetrain.getInstance().getOdometry().resetPosition(
        //         new Pose2d(trajectory.getInitialPose().getTranslation(), trajectory.getInitialPose().getRotation()),
        //         new Rotation2d(Math.toRadians(Drivetrain.getInstance().getPigeon().getFusedHeading())));

        //Set to x and y from starting Pose2d of path but keep current rotation value from odometry
        Pose2d initialPose = new Pose2d(trajectory.getInitialPose().getTranslation(), 
                Drivetrain.getInstance().getOdometry().getPoseMeters().getRotation());

        Rotation2d currentRot = Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading());

        Drivetrain.getInstance().getOdometry().resetPosition(initialPose, currentRot);
    }   
    
    @Override
    public void execute() {
        super.execute();
        
        double deltaT = timer.get();
        SmartDashboard.putNumber("Time", timer.get());
        Trajectory.State state = trajectory.sample(deltaT);

        Translation2d desiredTranslation = state.poseMeters.getTranslation();
        double desiredRotation = state.poseMeters.getRotation().getDegrees();

        Translation2d currentTranslation = Drivetrain.getInstance().getPose().getTranslation();
        double currentRotation = Drivetrain.getInstance().getPose().getRotation().getDegrees();

        SmartDashboard.putNumber("Trajectory X Error", desiredTranslation.getX() - currentTranslation.getX());
        SmartDashboard.putNumber("Trajectory Y Error", desiredTranslation.getY() - currentTranslation.getY());
        SmartDashboard.putNumber("Trajectory Angle Error", desiredRotation - currentRotation);

        SmartDashboard.putNumber("Trajectory X", desiredTranslation.getX());
        SmartDashboard.putNumber("Trajectory Y", desiredTranslation.getY());
        SmartDashboard.putNumber("Trajectory Angle", desiredRotation);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        timer.reset();
    }
}