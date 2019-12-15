package frc.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.SwerveModule;
import frc.robot.util.Vector;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.MathUtil;
import harkerrobolib.util.Conversions.PositionUnit;
import harkerrobolib.util.Conversions.SpeedUnit;
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
    public static final double ROTATION_MAGNITUDE = Math.sqrt(Math.pow(Drivetrain.DT_LENGTH, 2) + Math.pow(Drivetrain.DT_WIDTH, 2)); 

    private static final boolean IS_PERCENT_OUTPUT = false;

    private static final double FEET_PER_METER = 3.28;

    private double startTime;

    private int timeDur;

    private Trajectory traj;

    private int i;

    private double pigeonHeading;

    private Notifier n;

    private static final double PIGEON_KP_MP = 0.03;

    private static final double EPSILON = 1E-9;

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
                (double)timeDur / 1000, 
                Drivetrain.MAX_DRIVE_VELOCITY,  
                Drivetrain.MAX_DRIVE_ACCELERATION, 
                Drivetrain.MAX_DRIVE_JERK
        );

        long startTime = System.currentTimeMillis();
        traj = Pathfinder.generate(waypoints, config);
        SmartDashboard.putNumber("Path Generation Time", System.currentTimeMillis() - startTime);
        traj.get(0).heading = traj.get(1).heading;
        // SwerveModifier modifier = new SwerveModifier(traj);

        //Generate the individual wheel trajectories using the original trajectory as the center
        // modifier.modify(Drivetrain.DT_WIDTH, Drivetrain.DT_LENGTH, MODE);

        // traj = modifier.getFrontrajeftTrajectory();
        // tr = modifier.getFrontRightTrajectory();
        // bl = modifier.getBackLeftTrajectory();
        // br = modifier.getBackRightTrajectory();
    }
    
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("Traj len", traj.length());
        i = 0;//Resets the isfinished case to start again.

        pigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading();

        Drivetrain.getInstance().applyToAllAngle(
            (angleMotor) -> angleMotor.selectProfileSlot(Drivetrain.ANGLE_POSITION_SLOT, RobotMap.PRIMARY_INDEX)
        );

        Drivetrain.getInstance().applyToAllDrive(
            (driveMotor) -> driveMotor.selectProfileSlot(Drivetrain.DRIVE_VELOCITY_SLOT, RobotMap.PRIMARY_INDEX)
        );
        
        Drivetrain.getInstance().applyToAllDrive((driveMotor) -> driveMotor.setSelectedSensorPosition(0));

        // Drivetrain.getInstance().applyToAllDrive((driveMotors) -> driveMotors.configMotionProfileTrajectoryPeriod(timeDur));
        // Drivetrain.getInstance().applyToAllAngle((angleMotors) -> angleMotors.configMotionProfileTrajectoryPeriod(timeDur));

        // Drivetrain.getInstance().getTopLeft().getDriveMotor().startMotionProfile(trajDriveStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        // Drivetrain.getInstance().getTopLeft().getAngleMotor().startMotionProfile(trajAngleStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        
        // Drivetrain.getInstance().getTopRight().getDriveMotor().startMotionProfile(trDriveStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        // Drivetrain.getInstance().getTopRight().getAngleMotor().startMotionProfile(trAngleStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        
        // Drivetrain.getInstance().getBackLeft().getDriveMotor().startMotionProfile(blDriveStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        // Drivetrain.getInstance().getBackLeft().getAngleMotor().startMotionProfile(blAngleStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        
        // Drivetrain.getInstance().getBackRight().getDriveMotor().startMotionProfile(brDriveStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
        // Drivetrain.getInstance().getBackRight().getAngleMotor().startMotionProfile(brAngleStream, MIN_BUFFERED_POINTS, ControlMode.MotionProfile);
       


        double initialHeading = traj.get(0).heading;
        Vector turnAngle = new Vector(Math.cos(initialHeading) * EPSILON, Math.sin(initialHeading) * EPSILON);

        Drivetrain.getInstance().setDrivetrainVelocity(turnAngle, turnAngle, turnAngle, turnAngle, 0, true, true);

        boolean atSetpoint = false;
        long startTime = System.currentTimeMillis();
        while (!atSetpoint && System.currentTimeMillis() - startTime < 500)
        {
            atSetpoint = Math.abs(Drivetrain.getInstance().getTopLeft().getAngleDegrees() - initialHeading * 180 / Math.PI) > 3;
        }
        
        Runnable r = new Runnable() {

            Segment seg;
            double velocity, trajPosition, heading;
            double truePosition, error, constrainedError, output;
            double pigeonError, turnMagnitude, largestMag;
            Vector translation, tlRot, trRot, blRot, brRot;
            Vector sumTopLeft, sumTopRight, sumBackLeft, sumBackRight;

            @Override
            public void run() {

                if (i < traj.length()) {
                    seg = traj.get(i);
                    heading = seg.heading;
                    velocity = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, seg.velocity * FEET_PER_METER, SpeedUnit.ENCODER_UNITS) * Drivetrain.GEAR_RATIO;
                    trajPosition = Conversions.convertPosition(PositionUnit.FEET, seg.position * FEET_PER_METER, PositionUnit.ENCODER_UNITS) * Drivetrain.GEAR_RATIO;
                    
                    SmartDashboard.putNumber("Traj Pos", trajPosition);
                    SmartDashboard.putNumber("Traj Vel", velocity);
                    
                    truePosition = (Drivetrain.getInstance().getTopLeft().getDriveMotor().getSelectedSensorPosition()
                                            +  Drivetrain.getInstance().getTopRight().getDriveMotor().getSelectedSensorPosition()
                                            +  Drivetrain.getInstance().getBackLeft().getDriveMotor().getSelectedSensorPosition()
                                            +  Drivetrain.getInstance().getBackRight().getDriveMotor().getSelectedSensorPosition()) / 4.0;
                    
                    SmartDashboard.putNumber("True Pos", truePosition);
                    
                    error = Math.signum(trajPosition) * (Math.abs(trajPosition) - Math.abs(truePosition));
                    constrainedError = MathUtil.constrain(error, -8000, 8000);

                    SmartDashboard.putNumber("MP Error", constrainedError);

                    output = Drivetrain.DRIVE_MOTION_PROF_kF * velocity + Drivetrain.DRIVE_MOTION_PROF_kP * error + Drivetrain.DRIVE_MOTION_PROF_kA * seg.acceleration;
                    if (i < traj.length() - 1)
                        output += Drivetrain.DRIVE_MOTION_PROF_kS;
                    translation = new Vector(Math.cos(heading) * output, Math.sin(heading) * output);

                    pigeonError = pigeonHeading - Drivetrain.getInstance().getPigeon().getFusedHeading();
                    turnMagnitude = -PIGEON_KP_MP * pigeonError;
                    SmartDashboard.putNumber("Pigeon Error MP", pigeonError);
                    
                    //Scale by ROTATION_MAGNITUDE to make the magnitude of all vectors 1
                    //and then by turnMagnitude to reflect the desired rotational speed
                    tlRot = new Vector(Drivetrain.DT_LENGTH, Drivetrain.DT_WIDTH).scale(turnMagnitude / ROTATION_MAGNITUDE);
                    trRot = new Vector(Drivetrain.DT_LENGTH, -Drivetrain.DT_WIDTH).scale(turnMagnitude / ROTATION_MAGNITUDE);
                    blRot = new Vector(-Drivetrain.DT_LENGTH, Drivetrain.DT_WIDTH).scale(turnMagnitude / ROTATION_MAGNITUDE);
                    brRot = new Vector(-Drivetrain.DT_LENGTH, -Drivetrain.DT_WIDTH).scale(turnMagnitude / ROTATION_MAGNITUDE);
                    
                    sumTopLeft = Vector.add(tlRot, translation);
                    sumTopRight = Vector.add(trRot, translation);
                    sumBackLeft = Vector.add(blRot, translation);
                    sumBackRight = Vector.add(brRot, translation);
                    
                    // Scale down the vectors so that the largest possible magnitude is 1 (100% output)
                    largestMag = SwerveManualDDR.max4(sumTopLeft.getMagnitude(), sumTopRight.getMagnitude(), sumBackLeft.getMagnitude(), sumBackRight.getMagnitude());
                    
                    if(largestMag < 1) 
                        largestMag = 1; //Set to 1 so none of the vectors are modified

                    sumTopLeft.scale(1 / largestMag);
                    sumTopRight.scale(1 / largestMag);
                    sumBackLeft.scale(1 / largestMag);
                    sumBackRight.scale(1 / largestMag);
                    // move ks
                    Drivetrain.getInstance().setDrivetrainVelocity(sumTopLeft, sumTopRight, sumBackLeft, sumBackRight, 0, IS_PERCENT_OUTPUT, true);

                    i++;
                }
                SmartDashboard.putBoolean("Path Finished", i >= traj.length());
            }
        };
        n = new Notifier(r);
        n.startPeriodic((double)timeDur / 1000);
    }

    // private static BufferedTrajectoryPointStream createDriveStreamFromTrajectory(Trajectory traj) {
    //     BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();

    //     SmartDashboard.putNumber("Drive Points", traj.length());
    //     for (int i = 0; i < traj.length(); i++) {
    //         Segment seg = traj.get(i);
    //         TrajectoryPoint point = new TrajectoryPoint();

    //         point.position = Drivetrain.GEAR_RATIO * Conversions.convertPosition(PositionUnit.FEET, (seg.position*METERS_TO_FEET), PositionUnit.ENCODER_UNITS);
    //         point.velocity = Drivetrain.GEAR_RATIO * Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, (seg.velocity * METERS_TO_FEET), SpeedUnit.ENCODER_UNITS);
    //         point.profileSlotSelect0 = Drivetrain.DRIVE_MOTION_PROF_SLOT;
    //         point.isLastPoint = i == traj.length() - 1;
    //         point.arbFeedFwd = point.velocity > 0 ? Drivetrain.DRIVE_MOTION_PROF_kS : 0;
    //         point.timeDur = 0; //Set timeDur to zero because the Motion Profile period was already configured
    //         point.zeroPos = i == 0; //Zero on first point in profile

    //         stream.Write(point);
    //     }

    //     return stream;
    // }

    // private static BufferedTrajectoryPointStream createAngleStreamFromTrajectory(Trajectory traj, SwerveModule module) {
    //     BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();

    //     SmartDashboard.putNumber("Angle Points", traj.length());
    //     for (int i = 0; i < traj.length(); i++) {
    //         Segment seg = traj.get(i);
    //         TrajectoryPoint point = new TrajectoryPoint();

    //         SmartDashboard.putNumber("Heading", seg.heading);
    //         point.position = (Drivetrain.convertAngle(module, Math.toDegrees(seg.heading) / 360)) * 4096;
    //         point.profileSlotSelect0 = Drivetrain.ANGLE_MOTION_PROF_SLOT;
    //         point.isLastPoint = i == traj.length() - 1;
    //         point.arbFeedFwd = 0;
    //         point.timeDur = 0; //Set timeDur to zero because the Motion Profile period was already configured
    //         point.zeroPos = false; //Never change the angle encoder positions

    //         stream.Write(point);
    //     }

    //     return stream;
    // }

    @Override
    public void execute() {
        
        // SmartDashboard.putNumber("Top Right Velocity", Drivetrain.getInstance().getTopRight().getDriveMotor().getActiveTrajectoryVelocity());
        // SmartDashboard.putNumber("Top Right Angle Error", Drivetrain.getInstance().getTopRight().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("Top Right Drive Error", Drivetrain.getInstance().getTopRight().getDriveMotor().getClosedLoopError());

        // SmartDashboard.putNumber("Back Left Velocity", Drivetrain.getInstance().getBackLeft().getDriveMotor().getActiveTrajectoryVelocity());
        // SmartDashboard.putNumber("Back Left Angle Error", Drivetrain.getInstance().getBackLeft().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("Back Left Drive Error", Drivetrain.getInstance().getBackLeft().getDriveMotor().getClosedLoopError());

        // SmartDashboard.putNumber("Back Right Velocity", Drivetrain.getInstance().getBackRight().getDriveMotor().getActiveTrajectoryVelocity());
        // SmartDashboard.putNumber("Back Right Angle Error", Drivetrain.getInstance().getBackRight().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("Back Right Drive Error", Drivetrain.getInstance().getBackRight().getDriveMotor().getClosedLoopError());  
    }

    @Override
    public boolean isFinished() {
        // return false; //Holds Motion Profile Indefinitely, for testing
        return i >= traj.length();
    }

    @Override
    protected void interrupted() {
        Drivetrain.getInstance().applyToAllDrive((talon) -> talon.clearMotionProfileTrajectories());
        Drivetrain.getInstance().applyToAllAngle((talon) -> talon.clearMotionProfileTrajectories());

        end();
    }

    @Override
    protected void end() {
        SmartDashboard.putNumber("Path Total Time", Timer.getFPGATimestamp() - startTime);
        n.close();
    }
}