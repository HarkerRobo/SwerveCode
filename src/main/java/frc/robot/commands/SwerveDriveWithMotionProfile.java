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

    
    public SwerveDriveWithMotionProfile(double[][] path, double time) {
        requires(Drivetrain.getInstance());
        stream = new BufferedTrajectoryPointStream();
        setupTrajectoryStream(stream, path);
        Drivetrain.getInstance().applyToAllDrive((driveMotors) -> driveMotors.configMotionProfileTrajectoryPeriod(timeDur));
        timeTaken = leftPath.length * timeDur;
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
    /**
     * Determines the hypotenuse between the current point 
     * and the next point and sets that to be the position/speed
     */
    public void setupTrajectoryStreamDrive(BufferedStreamTrajectory stream, double[][] path) {
        for(int i = 1; i < path.length; i++) {
            
        }
    }

    /**
     * Determines the angle between the two points and sets that to be the desired angle
     */
    public void setupTrajectoryStreamAngle(BufferedStreamTrajectory stream, double[][] path) {
        for(int i = 0; i < path.length; i++) {
        }
    }

    @Override
    private void initialize() {
        Drivetrain.getInstance().applyToAllDrive((driveMotors) -> driveMotors.selectProfileSlot(Drivetrain.MOTION_PROF_SLOT, RobotMap.PRIMARY_PID_INDEX));
        applyToAllDrive((driveMotors) -> driveMotors.startMotionProfile(stream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile));
    }

    @Override
    private boolean isFinished() {
        return Drivetrain.getInstance().getTopLeft().isMotionProfileFinished();
    }


}