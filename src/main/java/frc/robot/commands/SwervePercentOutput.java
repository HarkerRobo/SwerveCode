package frc.robot.commands;

import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vector;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Controls the Swerve Modules using PercentOutput for the drive motors and Position PID for the angle motors.
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
 * @since 11/11/19
 */
public class SwervePercentOutput extends IndefiniteCommand {
    private static final double ROTATION_MAGNITUDE = Math.sqrt(Math.pow(Drivetrain.DT_LENGTH, 2) + Math.pow(Drivetrain.DT_WIDTH, 2)); 

    private static final double OUTPUT_MULTIPLIER = 0.2;
    
    public SwervePercentOutput() {
        requires(Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        Drivetrain.getInstance().applyToAllAngle(
            (motor) -> motor.selectProfileSlot(Drivetrain.ANGLE_POSITION_SLOT, RobotMap.PRIMARY_INDEX)
        );
    }

    @Override
    protected void execute() {
        double translateX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND);
        double translateY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND);
        double turnMagnitude = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND);

        Vector translation = new Vector(translateX, translateY);

        //initialize rotation vectors
        Vector[] rotationVectors = {new Vector(Drivetrain.DT_LENGTH, Drivetrain.DT_WIDTH),
                                    new Vector(Drivetrain.DT_LENGTH, -Drivetrain.DT_WIDTH),
                                    new Vector(-Drivetrain.DT_LENGTH, Drivetrain.DT_WIDTH),
                                    new Vector(-Drivetrain.DT_LENGTH, -Drivetrain.DT_WIDTH)};

        //Scale by ROTATION_MAGNITUDE to make the magnitude of all vectors 1
        //and then by turnMagnitude to reflect the desired rotational speed
        for(Vector vec : rotationVectors)
            vec.scale(turnMagnitude / ROTATION_MAGNITUDE);
        
        Vector[] combinedVectors = new Vector[rotationVectors.length];
        double largestMag = 0;

        //Combine rotation vectors with translation vectors and scale to output of left joystick
        for(int i = 0; i < 4; i++) {
            combinedVectors[i] = (Vector.add(rotationVectors[i], translation)).scale(OUTPUT_MULTIPLIER);
            if(combinedVectors[i].getMagnitude() > largestMag)
                largestMag = combinedVectors[i].getMagnitude(); //get largest magnitude out of all vectors
        }
		
		if(largestMag < 1)
			largestMag = 1; //Set to 1 so none of the vectors are modified
        
        //scale vectors
        for(Vector vec : combinedVectors)
            vec.scale(1 / largestMag);

        Drivetrain.getInstance().setDrivetrain(combinedVectors);
    }
    
    @Override
    protected void end() {
        Drivetrain.getInstance().stopAllDrive();
    }

    @Override
    protected void interrupted() {
        end();
    }
}