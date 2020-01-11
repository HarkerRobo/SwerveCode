package frc.robot.commands;

import java.util.List;
import java.util.ListIterator;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vector;

public class SwerveDriveWithOdometryProfiling extends CommandBase {

    private SwerveDriveOdometry odometry;
    private Trajectory trajectory;
    private List<Trajectory.State> trajectoryPoints;
    private int index;

    private static final double SPEED_MULTIPLIER = 0.5;
    
    public SwerveDriveWithOdometryProfiling(Pose2d startingPose, List<Translation2d> midpoints, Pose2d endingPose) {
        addRequirements(Drivetrain.getInstance());
        
        SwerveDriveKinematicsConstraint voltageConstraint = new SwerveDriveKinematicsConstraint(
                                            Drivetrain.getInstance().getKinematics(), Drivetrain.MP_MAX_DRIVE_VELOCITY);

        TrajectoryConfig config =
            new TrajectoryConfig(Drivetrain.MAX_DRIVE_VELOCITY,
                                Drivetrain.MAX_DRIVE_ACCELERATION)
                .setKinematics(Drivetrain.getInstance().getKinematics())
                .addConstraint(voltageConstraint);

        trajectory = TrajectoryGenerator.generateTrajectory(startingPose, midpoints, endingPose, config);
        trajectoryPoints = trajectory.getStates();
        odometry = Drivetrain.getInstance().getOdometry();
    }

    public SwerveDriveWithOdometryProfiling(List<Pose2d> poses) {
        addRequirements(Drivetrain.getInstance());
        
        SwerveDriveKinematicsConstraint voltageConstraint = new SwerveDriveKinematicsConstraint(
                                            Drivetrain.getInstance().getKinematics(), Drivetrain.MP_MAX_DRIVE_VELOCITY);

        TrajectoryConfig config =
            new TrajectoryConfig(Drivetrain.MAX_DRIVE_VELOCITY,
                                Drivetrain.MAX_DRIVE_ACCELERATION)
                .setKinematics(Drivetrain.getInstance().getKinematics())
                .addConstraint(voltageConstraint);

        trajectory = TrajectoryGenerator.generateTrajectory(poses, config);
        trajectoryPoints = trajectory.getStates();
        odometry = Drivetrain.getInstance().getOdometry();
    }

    public void initialize() {
        Rotation2d gyroAngle = Rotation2d.fromDegrees(-Drivetrain.getInstance().getPigeon().getFusedHeading());
        odometry.resetPosition(new Pose2d(), gyroAngle);
        index = 0;

        SmartDashboard.putNumber("Amount Trajectory Points", trajectoryPoints.size());
        SmartDashboard.putNumber("Trajectory Time", trajectory.getTotalTimeSeconds());
    }
        
    public void execute() {
        Trajectory.State state = trajectoryPoints.get(index);

        SmartDashboard.putNumber("Next Trajectory X Position", state.poseMeters.getTranslation().getX());
        SmartDashboard.putNumber("Next Trajectory Y Position", state.poseMeters.getTranslation().getY());
        SmartDashboard.putNumber("Next Trajectory Rotation", state.poseMeters.getRotation().getRadians());;
        
        Vector translationVector = new Vector(state.poseMeters.getTranslation().getX(), state.poseMeters.getTranslation().getY());
        Vector odometryCorrectionVector = new Vector(0, 0); // TODO
        Vector sum = translationVector.add(odometryCorrectionVector);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            sum.getX() * SPEED_MULTIPLIER, sum.getY() * SPEED_MULTIPLIER, state.poseMeters.getRotation().getRadians() * SPEED_MULTIPLIER, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading()));

        SwerveModuleState[] moduleStates = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);
        Drivetrain.getInstance().setDrivetrainVelocity(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3], 0, false, false);
                
        index++;
    }

    public boolean isFinished() {
        return index >= trajectoryPoints.size();
    }

    public void end(boolean interrupted)
    {
        
    }
    
}