package frc.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.SwerveModule;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.modifiers.SwerveModifier;

/**
 * Follows the specified trajectories by close looping and adding necessary Feed Forwards.
 * 
 * @author Shahzeb Lakhani
 * @author Chirag Kaushik
 * @author Angela Jia
 * @author Jatin Kohli
 * @since 11/13/19
 */
public class SwerveDriveWithMotionProfile extends Command {
    private static final SwerveModifier.Mode MODE = SwerveModifier.Mode.SWERVE_DEFAULT;

    private static final int MIN_BUFFERED_POINTS = 4;

    private int timeDur;

    private Trajectory tl;
    private Trajectory tr;
    private Trajectory bl;
    private Trajectory br;

    private BufferedTrajectoryPointStream tlDriveStream;
    private BufferedTrajectoryPointStream trDriveStream;    
    private BufferedTrajectoryPointStream blDriveStream;    
    private BufferedTrajectoryPointStream brDriveStream;

    /**
     * Generates a Trajectory from the Waypoints and creates the eight corresponding BufferedTrajectoryPointStreams
     * 
     * @param waypoints The Waypoints to generate the Trajectory from
     * @param timeDur The time in ms between each segment of the Trajectory
     */
    public SwerveDriveWithMotionProfile(Waypoint[] waypoints, int timeDur) {
        requires(Drivetrain.getInstance());
    
        this.timeDur = timeDur;

        Trajectory.Config config = new Trajectory.Config(
                Trajectory.FitMethod.HERMITE_QUINTIC,
                Trajectory.Config.SAMPLES_HIGH,
                timeDur, 
                Drivetrain.MAX_DRIVE_VELOCITY, 
                Drivetrain.MAX_DRIVE_ACCELERATION, 
                Drivetrain.MAX_DRIVE_JERK
        );

        long startTime = System.currentTimeMillis();
        Trajectory trajectory = Pathfinder.generate(waypoints, config);
        SmartDashboard.putNumber("Path Generation Time", System.currentTimeMillis() - startTime);

        SwerveModifier modifier = new SwerveModifier(trajectory);

        //Generate the individual wheel trajectories using the original trajectory as the center
        modifier.modify(Drivetrain.DT_WIDTH, Drivetrain.DT_LENGTH, MODE);

        Trajectory tl = modifier.getFrontLeftTrajectory();
        Trajectory tr = modifier.getFrontRightTrajectory();
        Trajectory bl = modifier.getBackLeftTrajectory();
        Trajectory br = modifier.getBackRightTrajectory();

        //Generate Drive Streams early, since they will always be the same
        tlDriveStream = createDriveStreamFromTrajectory(tl);
        trDriveStream = createDriveStreamFromTrajectory(tr);
        blDriveStream = createDriveStreamFromTrajectory(bl);
        brDriveStream = createDriveStreamFromTrajectory(br);
    }

    @Override
    public void initialize() {
        //Generate Angle Streams just before starting the profile, since every segment's position depends on the modules' current position
        BufferedTrajectoryPointStream tlAngleStream = createAngleStreamFromTrajectory(tl, Drivetrain.getInstance().getTopLeft());
        BufferedTrajectoryPointStream trAngleStream = createAngleStreamFromTrajectory(tr, Drivetrain.getInstance().getTopRight());
        BufferedTrajectoryPointStream blAngleStream = createAngleStreamFromTrajectory(bl, Drivetrain.getInstance().getBackLeft());
        BufferedTrajectoryPointStream brAngleStream = createAngleStreamFromTrajectory(br, Drivetrain.getInstance().getBackRight());

        Drivetrain.getInstance().applyToAllDrive((driveMotor) -> driveMotor.selectProfileSlot(Drivetrain.DRIVE_MOTION_PROF_SLOT, RobotMap.PRIMARY_INDEX));     
        Drivetrain.getInstance().applyToAllAngle((angleMotor) -> angleMotor.selectProfileSlot(Drivetrain.ANGLE_MOTION_PROF_SLOT, RobotMap.PRIMARY_INDEX));

        Drivetrain.getInstance().applyToAllDrive((driveMotors) -> driveMotors.configMotionProfileTrajectoryPeriod(timeDur));
        Drivetrain.getInstance().applyToAllAngle((angleMotors) -> angleMotors.configMotionProfileTrajectoryPeriod(timeDur));

        Drivetrain.getInstance().getTopLeft().getDriveMotor().startMotionProfile(tlDriveStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        Drivetrain.getInstance().getTopLeft().getAngleMotor().startMotionProfile(tlAngleStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        
        Drivetrain.getInstance().getTopRight().getDriveMotor().startMotionProfile(trDriveStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        Drivetrain.getInstance().getTopRight().getAngleMotor().startMotionProfile(trAngleStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        
        Drivetrain.getInstance().getBackLeft().getDriveMotor().startMotionProfile(blDriveStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        Drivetrain.getInstance().getBackLeft().getAngleMotor().startMotionProfile(blAngleStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        
        Drivetrain.getInstance().getBackRight().getDriveMotor().startMotionProfile(brDriveStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        Drivetrain.getInstance().getBackRight().getAngleMotor().startMotionProfile(brAngleStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
    }

    private static BufferedTrajectoryPointStream createDriveStreamFromTrajectory(Trajectory traj) {
        BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();

        for (int i = 0; i < traj.length(); i++) {
            Segment seg = traj.get(i);
            TrajectoryPoint point = new TrajectoryPoint();

            point.position = seg.position * Drivetrain.GEAR_RATIO;
            point.velocity = seg.velocity * Drivetrain.GEAR_RATIO;
            point.profileSlotSelect0 = Drivetrain.DRIVE_MOTION_PROF_SLOT;
            point.isLastPoint = i == traj.length() - 1;
            point.arbFeedFwd = 0;
            point.timeDur = 0; //Set timeDur to zero because the Motion Profile period was already configured
            point.zeroPos = i == 0; //Zero on first point in profile

            stream.Write(point);
        }

        return stream;
    }

    private static BufferedTrajectoryPointStream createAngleStreamFromTrajectory(Trajectory traj, SwerveModule module) {
        BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();

        for (int i = 0; i < traj.length(); i++) {
            Segment seg = traj.get(i);
            TrajectoryPoint point = new TrajectoryPoint();

            point.position = (Drivetrain.convertAngle(module, seg.heading) / 360) * 4096;
            point.profileSlotSelect0 = Drivetrain.ANGLE_MOTION_PROF_SLOT;
            point.isLastPoint = i == traj.length() - 1;
            point.arbFeedFwd = 0;
            point.timeDur = 0; //Set timeDur to zero because the Motion Profile period was already configured
            point.zeroPos = false; //Never change the angle encoder positions

            stream.Write(point);
        }

        return stream;
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
        return false; //Holds Motion Profile Indefinitely, for testing
        //return Drivetrain.getInstance().getTopLeft().getAngleMotor().isMotionProfileFinished() && Drivetrain.getInstance().getTopLeft().getDriveMotor().isMotionProfileFinished();
    }

    @Override
    protected void interrupted() {
        Drivetrain.getInstance().applyToAllDrive((talon) -> talon.clearMotionProfileTrajectories());
        Drivetrain.getInstance().applyToAllAngle((talon) -> talon.clearMotionProfileTrajectories());
    }
}