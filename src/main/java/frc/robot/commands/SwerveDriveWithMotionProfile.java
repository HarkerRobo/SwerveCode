package frc.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.modifiers.SwerveModifier;

/**
 * Follows the specified trajectories by close looping and adding necessary Feed Forwards.
 * 
 * @author Shahzeb Lakhani
 * @version 11/16/19
 */
public class SwerveDriveWithMotionProfile extends Command {
    private static final int MIN_BUFFERED_POINTS = 4;
    private static final SwerveModifier.Mode MODE = SwerveModifier.Mode.SWERVE_DEFAULT;
    private static final double WHEELBASE_WIDTH = Drivetrain.DT_WIDTH * 0.0254; // in to m
    private static final double WHEELBASE_DEPTH = Drivetrain.DT_LENGTH * 0.0254; // in to m

    private Trajectory tl;
    private Trajectory tr;
    private Trajectory bl;
    private Trajectory br;
    private BufferedTrajectoryPointStream tlAngleStream;
    private BufferedTrajectoryPointStream tlDriveStream;
    private BufferedTrajectoryPointStream trAngleStream;
    private BufferedTrajectoryPointStream trDriveStream;
    private BufferedTrajectoryPointStream blAngleStream;
    private BufferedTrajectoryPointStream blDriveStream;
    private BufferedTrajectoryPointStream brAngleStream;
    private BufferedTrajectoryPointStream brDriveStream;


    public SwerveDriveWithMotionProfile(int timeDur, Waypoint[] waypoints) {
        requires(Drivetrain.getInstance());
        
        Drivetrain.getInstance().applyToAllDrive((driveMotors) -> driveMotors.configMotionProfileTrajectoryPeriod(timeDur));

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
        Trajectory trajectory = Pathfinder.generate(waypoints, config);

        SwerveModifier modifier = new SwerveModifier(trajectory);

        // Generate the individual wheel trajectories using the original trajectory
        // as the centre
        modifier.modify(WHEELBASE_WIDTH, WHEELBASE_DEPTH, MODE);

        Trajectory tl = modifier.getFrontLeftTrajectory();       // Get the Front Left wheel
        Trajectory tr = modifier.getFrontRightTrajectory();      // Get the Front Right wheel
        Trajectory bl = modifier.getBackLeftTrajectory();        // Get the Back Left wheel
        Trajectory br = modifier.getBackRightTrajectory();

        tlAngleStream = createAngleStreamFromTrajectory(tl);
        tlDriveStream = createDriveStreamFromTrajectory(tl);
        trAngleStream = createAngleStreamFromTrajectory(tr);
        trDriveStream = createDriveStreamFromTrajectory(tr);
        blAngleStream = createAngleStreamFromTrajectory(bl);
        blDriveStream = createDriveStreamFromTrajectory(bl);
        brAngleStream = createAngleStreamFromTrajectory(br);
        brDriveStream = createDriveStreamFromTrajectory(br);

    }

    private static BufferedTrajectoryPointStream createDriveStreamFromTrajectory(Trajectory traj) {
        BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();
        for (int i = 0; i < traj.length(); i++) {
            Segment seg = traj.get(i);
            TrajectoryPoint point = new TrajectoryPoint();

            point.position = seg.position;
            point.velocity = seg.velocity;
            point.isLastPoint = i == traj.length() - 1;
            point.arbFeedFwd = 0;
            point.timeDur = 0;
            point.zeroPos = i == 0;

            stream.Write(point);
        }
        return stream;
    }

    private static BufferedTrajectoryPointStream createAngleStreamFromTrajectory(Trajectory traj) {
        BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();
        for (int i = 0; i<traj.length(); i++) {
            Segment seg = traj.get(i);
            TrajectoryPoint point = new TrajectoryPoint();

            point.position = seg.heading * 4096 / 360;
            point.arbFeedFwd = 0;
            point.timeDur = 0;
            point.zeroPos =
             i == 0;

            stream.Write(point);
        }
        return stream;
    }

    @Override
    public void initialize() {
        Drivetrain.getInstance().applyToAllAngle((angleMotor) -> angleMotor.selectProfileSlot(Drivetrain.ANGLE_MOTION_PROF_SLOT, RobotMap.PRIMARY_INDEX));
        Drivetrain.getInstance().applyToAllDrive((driveMotor) -> driveMotor.selectProfileSlot(Drivetrain.DRIVE_MOTION_PROF_SLOT, RobotMap.PRIMARY_INDEX));     
        
        Drivetrain.getInstance().getTopLeft().getAngleMotor().startMotionProfile(tlAngleStream, MIN_BUFFERED_PTS, ControlMode.MotionProfile);
        Drivetrain.getInstance().getTopLeft().getDriveMotor().startMotionProfile(tlDriveStream, MIN_BUFFERED_PTS, ControlMode.MotionProfile);
        
        Drivetrain.getInstance().getTopRight().getAngleMotor().startMotionProfile(trAngleStream, MIN_BUFFERED_PTS, ControlMode.MotionProfile);
        Drivetrain.getInstance().getTopRight().getDriveMotor().startMotionProfile(trDriveStream, MIN_BUFFERED_PTS, ControlMode.MotionProfile);
        
        Drivetrain.getInstance().getBackLeft().getAngleMotor().startMotionProfile(blAngleStream, MIN_BUFFERED_PTS, ControlMode.MotionProfile);
        Drivetrain.getInstance().getBackLeft().getDriveMotor().startMotionProfile(blDriveStream, MIN_BUFFERED_PTS, ControlMode.MotionProfile);
        
        Drivetrain.getInstance().getBackRight().getAngleMotor().startMotionProfile(brAngleStream, MIN_BUFFERED_PTS, ControlMode.MotionProfile);
        Drivetrain.getInstance().getBackRight().getDriveMotor().startMotionProfile(brDriveStream, MIN_BUFFERED_PTS, ControlMode.MotionProfile);
   
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Top Left Velocity", Drivetrain.getInstance().getTopLeft().getDriveMotor().getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Top Left Angle Error", Drivetrain.getInstance().getTopLeft().getAngleMotor().getClosedLoopError());
        SmartDashboard.putNumber("Top Left Drive Error", Drivetrain.getInstance().getTopLeft().getDriveMotor().getClosedLoopError());
        
        SmartDashboard.putNumber("Top Right Velocity", Drivetrain.getInstance().getTopRight().getDriveMotor().getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Top Right Angle Error", Drivetrain.getInstance().getTopRight().getAngleMotor().getClosedLoopError());
        SmartDashboard.putNumber("Top Right Drive Error", Drivetrain.getInstance().getTopRight().getDriveMotor().getClosedLoopError());

        SmartDashboard.putNumber("Back Left Velocity", Drivetrain.getInstance().getBackLeft().getDriveMotor().getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Back Left Angle Error", Drivetrain.getInstance().getBackLeft().getAngleMotor().getClosedLoopError());
        SmartDashboard.putNumber("Back Left Drive Error", Drivetrain.getInstance().getBackLeft().getDriveMotor().getClosedLoopError());

        SmartDashboard.putNumber("Back Right Velocity", Drivetrain.getInstance().getBackRight().getDriveMotor().getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Back Right Angle Error", Drivetrain.getInstance().getBackRight().getAngleMotor().getClosedLoopError());
        SmartDashboard.putNumber("Back Right Drive Error", Drivetrain.getInstance().getBackRight().getDriveMotor().getClosedLoopError());
    }

    @Override
    public boolean isFinished() {
        return Drivetrain.getInstance().getTopLeft().getAngleMotor().isMotionProfileFinished() && Drivetrain.getInstance().getTopLeft().getDriveMotor().isMotionProfileFinished();
    }


}