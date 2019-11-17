package frc.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.SwerveModifier;

/**
 * Follows the specified trajectories by close looping and adding necessary Feed Forwards.
 * 
 * @author Shahzeb Lakhani
 * @version 11/13/19
 */
public class SwerveDriveWithMotionProfile extends Command {
    private static final int MIN_BUFFERED_POINTS = 4;
    private static final SwerveModifier.Mode MODE = SwerveModifier.Mode.SWERVE_DEFAULT;
    private static final double WHEELBASE_WIDTH = Drivetrain.DT_WIDTH * 0.0254; // in to m
    private static final double WHEELBASE_DEPTH = Drivetrain.DT_LENGTH * 0.0254; // in to m
    private Trajectory fl;
    private Trajectory fr;
    private Trajectory bl;
    private Trajectory br;


    public SwerveDriveWithMotionProfile(int timeDur, Waypoint[] waypoints) {
        requires(Drivetrain.getInstance());
        
        Drivetrain.getInstance().applyToAllDrive((driveMotors) -> driveMotors.configMotionProfileTrajectoryPeriod(timeDur));

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
        Trajectory trajectory = Pathfinder.generate(waypoints, config);

        SwerveModifier modifier = new SwerveModifier(trajectory);

        // Generate the individual wheel trajectories using the original trajectory
        // as the centre
        modifier.modify(WHEELBASE_WIDTH, WHEELBASE_DEPTH, MODE);

        Trajectory fl = modifier.getFrontLeftTrajectory();       // Get the Front Left wheel
        Trajectory fr = modifier.getFrontRightTrajectory();      // Get the Front Right wheel
        Trajectory bl = modifier.getBackLeftTrajectory();        // Get the Back Left wheel
        Trajectory br = modifier.getBackRightTrajectory();
    }

    @Override
    public void initialize() {
        // Drivetrain.getInstance().applyToAllAngle((angleMotor) -> angleMotor.selectProfileSlot(Drivetrain.DRIVE_MOTION_PROF_SLOT, RobotMap.PRIMARY_INDEX));
        // Drivetrain.getInstance().applyToAllDrive((driveMotor) -> driveMotor.selectProfileSlot(Drivetrain.DRIVE_MOTION_PROF_SLOT, RobotMap.PRIMARY_INDEX));
    
        
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // return Drivetrain.getInstance().getTopLeft().isMotionProfileFinished();
        return Drivetrain.getInstance().getTopLeft().getAngleMotor().isMotionProfileFinished() && Drivetrain.getInstance().getTopLeft().getDriveMotor().isMotionProfileFinished();
    }


}