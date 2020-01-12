package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.commands.SwerveDriveWithOdometryProfiling;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.HSDDRPad;
import harkerrobolib.wrappers.XboxGamepad;

/**
 * Contains controllers (Xbox or DDR) and their button bindings
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @since 11/1/19
 */
public class OI {
    private static OI instance;

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;
    private static final int DDR_PORT = 2;

    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;

    private XboxGamepad operatorGamepad;
    private XboxGamepad driverGamepad;
    private HSDDRPad ddrGamepad;

    private SwerveDriveKinematicsConstraint constraint;

    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);
        ddrGamepad = new HSDDRPad(DDR_PORT);

        initBindings();
    }

    public void initBindings() {
        constraint = new SwerveDriveKinematicsConstraint(Drivetrain.getInstance().getKinematics(), Drivetrain.MAX_DRIVE_VELOCITY);

        TrajectoryConfig config = new TrajectoryConfig(Drivetrain.MAX_DRIVE_VELOCITY, Drivetrain.MAX_DRIVE_ACCELERATION)
                .setKinematics(Drivetrain.getInstance().getKinematics())
                .addConstraint(constraint);

        Trajectory linearTrajectory = TrajectoryGenerator.generateTrajectory(List.of( 
            new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
            new Pose2d(0, 5, Rotation2d.fromDegrees(90))), config);

        Trajectory circle = TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(2, 2, Rotation2d.fromDegrees(90)),
            new Pose2d(0, 4, Rotation2d.fromDegrees(180)),
            new Pose2d(-2, 2, Rotation2d.fromDegrees(-90)),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0))), config);

        driverGamepad.getButtonA().whenPressed(new SwerveDriveWithOdometryProfiling(linearTrajectory));
        driverGamepad.getButtonB().whenPressed(new SwerveDriveWithOdometryProfiling(circle));
    }

    public XboxGamepad getDriverGamepad() {
        return driverGamepad;
    }

    public XboxGamepad getOperatorGamepad() {
        return operatorGamepad;
    }

    public HSDDRPad getDDRGamepad() {
        return ddrGamepad;
    }
    
    public static OI getInstance() {
        if(instance == null)
            instance = new OI();
        return instance;
    }
}