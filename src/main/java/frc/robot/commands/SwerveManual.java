
package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.MathUtil;

/**
 * Controls the Swerve Modules using PercentOutput or Velociy for the drive motors and 
 * Position PID for the angle motors.
 * The left joystick controls translation (velocity direction and magnitude)
 * The right joystick's X axis controls rotation (angular velocity magnitude)
 * 
 * 'back' is defined as closest to the battery
 * 'left' is defined as left when standing at the back and looking forward
 * 
 * @author Chirag Kaushik
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Arjun Dixit
 * @since 11/4/19
 */
public class SwerveManual extends CommandBase {
    public static final double ROTATION_MAGNITUDE = Math.sqrt(Math.pow(Drivetrain.DT_LENGTH, 2) + Math.pow(Drivetrain.DT_WIDTH, 2)); 
    private static final double OUTPUT_MULTIPLIER = 0.5;
    private static final double VELOCITY_HEADING_MULTIPLIER = 70;
    private static final boolean IS_PERCENT_OUTPUT = false;

    private SwerveDriveKinematics swerve = new SwerveDriveKinematics(
                                                        Drivetrain.FRONT_LEFT_LOCATION,
                                                        Drivetrain.FRONT_RIGHT_LOCATION,
                                                        Drivetrain.BACK_LEFT_LOCATION,
                                                        Drivetrain.BACK_RIGHT_LOCATION);

    private double translateX, translateY, turnMagnitude;
    
    private static double prevPigeonHeading;
    private static long prevTime;
    
    private static boolean pigeonFlag; //True if the Driver Right X input is non-zero
    private static double pigeonAngle;
    
    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());

        pigeonFlag = false;
        pigeonAngle = 0;
        prevPigeonHeading = 0;
        prevTime = System.currentTimeMillis();
    }
    

    @Override
    public void initialize() {
        Drivetrain.getInstance().applyToAllAngle(
            (angleMotor) -> angleMotor.selectProfileSlot(Drivetrain.ANGLE_POSITION_SLOT, RobotMap.PRIMARY_INDEX)
        );

        Drivetrain.getInstance().applyToAllDrive(
            (driveMotor) -> driveMotor.selectProfileSlot(Drivetrain.DRIVE_VELOCITY_SLOT, RobotMap.PRIMARY_INDEX)
        );
    }

    @Override
    public void execute() {
        translateX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND);
        translateY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND);
        turnMagnitude = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND);

        //scale input from joysticks
        translateX = translateX * Drivetrain.MAX_DRIVE_VELOCITY;
        translateY = translateY * Drivetrain.MAX_DRIVE_VELOCITY;
        turnMagnitude = turnMagnitude * Drivetrain.MAX_ROTATION_VELOCITY;

        // Drivetrain.getInstance().setSwerveModuleVelocity(Drivetrain.getInstance().getTopLeft(), moduleStates[0].speedMetersPerSecond, moduleStates[0].angle.getDegrees(), false, false);
        // Drivetrain.getInstance().setSwerveModuleVelocity(Drivetrain.getInstance().getTopRight(), moduleStates[1].speedMetersPerSecond, moduleStates[1].angle.getDegrees(), false, false);
        // Drivetrain.getInstance().setSwerveModuleVelocity(Drivetrain.getInstance().getBackLeft(), moduleStates[2].speedMetersPerSecond, moduleStates[2].angle.getDegrees(), false, false);
        // Drivetrain.getInstance().setSwerveModuleVelocity(Drivetrain.getInstance().getBackRight(), moduleStates[3].speedMetersPerSecond, moduleStates[3].angle.getDegrees(), false, false);

        // double currentPigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading();

        // if(pigeonFlag && turnMagnitude == 0) { //If there was joystick input but now there is not
        //     long currentTime = System.currentTimeMillis();
        //     double deltaTime = (double)(currentTime - prevTime);
        //     double turnVel = (currentPigeonHeading - prevPigeonHeading) / deltaTime;
        //     pigeonAngle = currentPigeonHeading + turnVel * VELOCITY_HEADING_MULTIPLIER; // account for momentum when turning
        // }

        // pigeonFlag = Math.abs(turnMagnitude) > 0; //Update pigeon flag

        // if(!pigeonFlag) { //If there is no joystick input currently
        //     turnMagnitude = -Drivetrain.PIGEON_kP * (pigeonAngle - currentPigeonHeading);
        //     SmartDashboard.putNumber("Pigeon Error", pigeonAngle - currentPigeonHeading);
        // }

        // prevPigeonHeading = currentPigeonHeading;
        // prevTime = System.currentTimeMillis();

        // ChassisSpeeds speed = new ChassisSpeeds(translateX, translateY, turnMagnitude);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translateX, translateY, turnMagnitude, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())
        );

        // Now use this in our kinematics
        SwerveModuleState[] moduleStates = swerve.toSwerveModuleStates(speeds);

        // SwerveModuleState[] moduleStates = swerve.toSwerveModuleStates(speed);

        // Vector translation = new Vector(translateX, translateY);

        // //Adjust for field sensitive drive using pigeon
        // if(Drivetrain.getInstance().isFieldSensitive()) {
        //     translation.rotate(-Drivetrain.getInstance().getPigeon().getFusedHeading());
        // }

        // Vector topLeftRotation = new Vector(Drivetrain.DT_LENGTH, Drivetrain.DT_WIDTH);
        // Vector topRightRotation = new Vector(Drivetrain.DT_LENGTH, -Drivetrain.DT_WIDTH);
        // Vector backLeftRotation = new Vector(-Drivetrain.DT_LENGTH, Drivetrain.DT_WIDTH);
        // Vector backRightRotation = new Vector(-Drivetrain.DT_LENGTH, -Drivetrain.DT_WIDTH);

        // //Scale by ROTATION_MAGNITUDE to make the magnitude of all vectors 1
        // //and then by turnMagnitude to reflect the desired rotational speed
        // topLeftRotation.scale(turnMagnitude / ROTATION_MAGNITUDE);
        // topRightRotation.scale(turnMagnitude / ROTATION_MAGNITUDE);
        // backLeftRotation.scale(turnMagnitude / ROTATION_MAGNITUDE);
        // backRightRotation.scale(turnMagnitude / ROTATION_MAGNITUDE);
        
        // Vector sumTopLeft = Vector.add(topLeftRotation, translation).scale(OUTPUT_MULTIPLIER);
        // Vector sumTopRight = Vector.add(topRightRotation, translation).scale(OUTPUT_MULTIPLIER);
        // Vector sumBackLeft = Vector.add(backLeftRotation, translation).scale(OUTPUT_MULTIPLIER);
        // Vector sumBackRight = Vector.add(backRightRotation, translation).scale(OUTPUT_MULTIPLIER);
        
        // // Scale down the vectors so that the largest possible magnitude is 1 (100% output)
        // double largestMag = max4(sumTopLeft.getMagnitude(), sumTopRight.getMagnitude(), sumBackLeft.getMagnitude(), sumBackRight.getMagnitude());
        
        // if(largestMag < 1) 
        //     largestMag = 1; //Set to 1 so none of the vectors are modified

        // sumTopLeft.scale(1 / largestMag);
        // sumTopRight.scale(1 / largestMag);
        // sumBackLeft.scale(1 / largestMag);
        // sumBackRight.scale(1 / largestMag);

        Drivetrain.getInstance().setDrivetrainVelocity(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3], 0, IS_PERCENT_OUTPUT, false);
    }

    public static double max4(double a, double b, double c, double d) {
        return Math.max(Math.max(a, b), Math.max(c, d));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().stopAllDrive();
    }
}