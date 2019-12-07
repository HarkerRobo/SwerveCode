package frc.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.SwerveModule;
import frc.robot.util.Vector;
import harkerrobolib.util.Conversions;
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
    private static final SwerveModifier.Mode MODE = SwerveModifier.Mode.SWERVE_DEFAULT;

    private static final boolean IS_PERCENT_OUTPUT = false;

    private static final int MIN_BUFFERED_POINTS = 4;

    private static final double FEET_PER_METER = 3.28;

    private int timeDur;

    private Trajectory traj;

    private int i;

    private double pigeonHeading;

    public static final double ROTATION_MAGNITUDE = Math.sqrt(Math.pow(Drivetrain.DT_LENGTH, 2) + Math.pow(Drivetrain.DT_WIDTH, 2)); 
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
                Trajectory.Config.SAMPLES_FAST,
                (double)timeDur / 1000, 
                Drivetrain.MAX_DRIVE_VELOCITY,  
                Drivetrain.MAX_DRIVE_ACCELERATION, 
                Drivetrain.MAX_DRIVE_JERK
        );

        long startTime = System.currentTimeMillis();
        traj = Pathfinder.generate(waypoints, config);
        SmartDashboard.putNumber("Path Generation Time", System.currentTimeMillis() - startTime);

        SwerveModifier modifier = new SwerveModifier(traj);

        //Generate the individual wheel trajectories using the original trajectory as the center
        modifier.modify(Drivetrain.DT_WIDTH, Drivetrain.DT_LENGTH, MODE);

        // traj = modifier.getFrontrajeftTrajectory();
        // tr = modifier.getFrontRightTrajectory();
        // bl = modifier.getBackLeftTrajectory();
        // br = modifier.getBackRightTrajectory();
    }
    
    @Override
    public void initialize() {
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
        
        Runnable r = new Runnable() {
            // double prevPosition = 0;
            @Override
            public void run() {
                if (i < traj.length()) {
                    Segment seg = traj.get(i);
                    double heading = seg.heading;
                    double velocity = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, seg.velocity * FEET_PER_METER, SpeedUnit.ENCODER_UNITS) * Drivetrain.GEAR_RATIO;
                    double trajPosition = Conversions.convertPosition(PositionUnit.FEET, seg.position * FEET_PER_METER, PositionUnit.ENCODER_UNITS) * Drivetrain.GEAR_RATIO;

                    double truePosition = (Drivetrain.getInstance().getTopLeft().getDriveMotor().getSelectedSensorPosition()
                                            +  Drivetrain.getInstance().getTopRight().getDriveMotor().getSelectedSensorPosition()
                                            +  Drivetrain.getInstance().getBackLeft().getDriveMotor().getSelectedSensorPosition()
                                            +  Drivetrain.getInstance().getBackRight().getDriveMotor().getSelectedSensorPosition()) / 4.0;
                    
                    double error = trajPosition - truePosition;
                    double output = Drivetrain.DRIVE_MOTION_PROF_kF * velocity + Drivetrain.DRIVE_MOTION_PROF_kP * error;
                    output = Math.signum(output) * Math.sqrt(output);

                    Vector translation = new Vector(Math.cos(heading) * output, Math.sin(heading) * output);

                    // prevPosition = trajPosition;

                    Vector topLeftRotation = new Vector(Drivetrain.DT_LENGTH, Drivetrain.DT_WIDTH);
                    Vector topRightRotation = new Vector(Drivetrain.DT_LENGTH, -Drivetrain.DT_WIDTH);
                    Vector backLeftRotation = new Vector(-Drivetrain.DT_LENGTH, Drivetrain.DT_WIDTH);
                    Vector backRightRotation = new Vector(-Drivetrain.DT_LENGTH, -Drivetrain.DT_WIDTH);

                    double turnMagnitude = -Drivetrain.PIGEON_kP * (pigeonHeading - Drivetrain.getInstance().getPigeon().getFusedHeading());

                    //Scale by ROTATION_MAGNITUDE to make the magnitude of all vectors 1
                    //and then by turnMagnitude to reflect the desired rotational speed
                    topLeftRotation.scale(turnMagnitude / ROTATION_MAGNITUDE);
                    topRightRotation.scale(turnMagnitude / ROTATION_MAGNITUDE);
                    backLeftRotation.scale(turnMagnitude / ROTATION_MAGNITUDE);
                    backRightRotation.scale(turnMagnitude / ROTATION_MAGNITUDE);
                    
                    Vector sumTopLeft = Vector.add(topLeftRotation, translation);
                    Vector sumTopRight = Vector.add(topRightRotation, translation);
                    Vector sumBackLeft = Vector.add(backLeftRotation, translation);
                    Vector sumBackRight = Vector.add(backRightRotation, translation);
                    
                    // Scale down the vectors so that the largest possible magnitude is 1 (100% output)
                    double largestMag = SwerveManual.max4(sumTopLeft.getMagnitude(), sumTopRight.getMagnitude(), sumBackLeft.getMagnitude(), sumBackRight.getMagnitude());
                    
                    if(largestMag < 1) 
                        largestMag = 1; //Set to 1 so none of the vectors are modified

                    sumTopLeft.scale(1 / largestMag);
                    sumTopRight.scale(1 / largestMag);
                    sumBackLeft.scale(1 / largestMag);
                    sumBackRight.scale(1 / largestMag);

                    Drivetrain.getInstance().setDrivetrainVelocity(sumTopLeft, sumTopRight, sumBackLeft, sumBackRight, i > 0 ? Drivetrain.DRIVE_MOTION_PROF_kS : 0, IS_PERCENT_OUTPUT);
                    i++;  

                    double trajDesiredPos = Conversions.convertPosition(PositionUnit.FEET, trajPosition * FEET_PER_METER, PositionUnit.ENCODER_UNITS) * Drivetrain.GEAR_RATIO;
                    double trajActualPos = Drivetrain.getInstance().getTopLeft().getDriveMotor().getSelectedSensorPosition();

                    SmartDashboard.putNumber("Pos Error", Math.abs(trajDesiredPos) - Math.abs(trajActualPos));
                    SmartDashboard.putNumber("Vel Error", Drivetrain.getInstance().getTopLeft().getDriveMotor().getClosedLoopError());
                    SmartDashboard.putNumber("Desired Vel", Drivetrain.getInstance().getTopLeft().getDriveMotor().getClosedLoopTarget());
                }
                SmartDashboard.putBoolean("Path Finished", i >= traj.length());
            }
        };
        Notifier n = new Notifier(r);
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
        SmartDashboard.putNumber("Top Left Velocity", Drivetrain.getInstance().getTopLeft().getDriveMotor().getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Top Left Angle Error", Drivetrain.getInstance().getTopLeft().getAngleMotor().getClosedLoopError());
        SmartDashboard.putNumber("Top Left Drive Error", Drivetrain.getInstance().getTopLeft().getDriveMotor().getClosedLoopError());
        
        // SmartDashboard.putNumber("Top Right Velocity", Drivetrain.getInstance().getTopRight().getDriveMotor().getActiveTrajectoryVelocity());
        // SmartDashboard.putNumber("Top Right Angle Error", Drivetrain.getInstance().getTopRight().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("Top Right Drive Error", Drivetrain.getInstance().getTopRight().getDriveMotor().getClosedLoopError());

        // SmartDashboard.putNumber("Back Left Velocity", Drivetrain.getInstance().getBackLeft().getDriveMotor().getActiveTrajectoryVelocity());
        // SmartDashboard.putNumber("Back Left Angle Error", Drivetrain.getInstance().getBackLeft().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("Back Left Drive Error", Drivetrain.getInstance().getBackLeft().getDriveMotor().getClosedLoopError());

        // SmartDashboard.putNumber("Back Right Velocity", Drivetrain.getInstance().getBackRight().getDriveMotor().getActiveTrajectoryVelocity());
        // SmartDashboard.putNumber("Back Right Angle Error", Drivetrain.getInstance().getBackRight().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("Back Right Drive Error", Drivetrain.getInstance().getBackRight().getDriveMotor().getClosedLoopError());
    
        MotionProfileStatus trajDriveStatus = new MotionProfileStatus();
        MotionProfileStatus trajAngleStatus = new MotionProfileStatus();
        Drivetrain.getInstance().getTopLeft().getAngleMotor().getMotionProfileStatus(trajAngleStatus);
        Drivetrain.getInstance().getTopLeft().getDriveMotor().getMotionProfileStatus(trajDriveStatus);

        // System.out.printrajn(trajDriveStatus.btmBufferCnt);
        SmartDashboard.putNumber("traj Drive Top Buffer", trajDriveStatus.topBufferCnt);
        SmartDashboard.putNumber("traj Drive Btm Buffer", trajDriveStatus.btmBufferCnt);
        SmartDashboard.putNumber("traj Angle Top Buffer", trajAngleStatus.topBufferCnt);
        SmartDashboard.putNumber("traj Angle Btm Buffer", trajAngleStatus.btmBufferCnt);
        SmartDashboard.putBoolean("traj Drive Has Overrun", trajDriveStatus.hasUnderrun);
        SmartDashboard.putBoolean("traj Angle Has Overrun", trajAngleStatus.hasUnderrun);
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
    }
}