package frc.robot.commands;
import frc.robot.subsystems.Drivetrain;

/**
 * Follows the specified trajectories by close looping and adding necessary Feed Forwards.
 * 
 * @author Shahzeb Lakhani
 * @version 11/13/19
 */
public class SwerveDriveWithMotionProfile extends Command {
    private static final int MIN_BUFFERED_POINTS = 4;
    private double leftLastSetpoint;
    private double rightLastSetpoint;
    private BufferedStreamTrajectory stream;
    
    
    public SwerveDriveWithMotionProfile(double time) {
        requires(Drivetrain.getInstance());
        
        Drivetrain.getInstance().applyToAllDrive((driveMotors) -> driveMotors.configMotionProfileTrajectoryPeriod(time));
        timeTaken = leftPath.length * timeDur;
        Waypoint[] points = new Waypoint[] {
            new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
            new Waypoint(-2, -2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
            new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
        };
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
        Trajectory trajectory = Pathfinder.generate(points, config);
    }

    /**
     * public void setupTrajectoryStream(BufferedStreamTrajectory stream, double[][] path) {
        for(int i = 0; i < path.length; i++) {
            TrajectoryPoint point = new TrajectoryPoint();
            point.position = Conversions.convert(PositionUnit.FEET, path[i][0], PositionUnit.ENCODER_UNITS);
            point.velocity = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, path[i][1], SpeedUnit.ENCODER_UNITS);
            point.arbFeedFwd = path[i][2]*Drivetrain.MOTION_PROF_DRIVE_kA+ Math.signum(point.velocity) * Drivetrain.MOTION_PROF_DRIVE_kS);
            if(i == 0) 
                point.zeroPos = true;
            else if(i == path.length-1) {
                point.isLastPoint = true;
                if (isLeft)
                    leftLastSetpoint = point.position;
                else
                    rightLastSetpoint = point.position;
            }
            point.profileSlotSelect0 = Drivetrain.MOTION_PROF_SLOT;
            point.timeDur = 0;
            stream.Write(point);
        }
    }
    
    */

    @Override
    private void initialize() {
        for (int i = 0; i < trajectory.length(); i++) {
            Trajectory.Segment seg = trajectory.get(i);
            
            System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
                seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
                    seg.acceleration, seg.jerk, seg.heading);
        }    

        double wheelbase_width = 0.6;

        // The distance between the front and back sides of the wheelbase is 0.5m
        double wheelbase_depth = 0.5;

        // The swerve mode to generate will be the 'default' mode, where the 
        // robot will constantly be facing forward and 'sliding' sideways to 
        // follow a curved path.
        SwerveModifier.Mode mode = SwerveModifier.Mode.SWERVE_DEFAULT;

        // Create the Modifier Object
        SwerveModifier modifier = new SwerveModifier(trajectory);

        // Generate the individual wheel trajectories using the original trajectory
        // as the centre
        modifier.modify(wheelbase_width, wheelbase_depth, mode);

        Trajectory fl = modifier.getFrontLeftTrajectory();       // Get the Front Left wheel
        Trajectory fr = modifier.getFrontRightTrajectory();      // Get the Front Right wheel
        Trajectory bl = modifier.getBackLeftTrajectory();        // Get the Back Left wheel
        Trajectory br = modifier.getBackRightTrajectory();
    }

    @Override
    private boolean isFinished() {
        return Drivetrain.getInstance().getTopLeft().isMotionProfileFinished();
    }


}