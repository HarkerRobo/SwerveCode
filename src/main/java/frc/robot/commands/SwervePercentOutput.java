package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vector;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SwervePercentOutput extends IndefiniteCommand {
    private static final double ROOT = Math.sqrt(Math.pow(Drivetrain.DT_LENGTH, 2) + Math.pow(Drivetrain.DT_WIDTH, 2)); 
    private static final double OUTPUT_MULTIPLIER = 0.2;
    
    public SwervePercentOutput() {
        requires(Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        Drivetrain.getInstance().applyToAllAngle((motor) -> motor.selectProfileSlot(Drivetrain.ANGLE_POSITION_SLOT, 0));
    }

    @Override
    protected void execute() {
        double translateX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND);
        double translateY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND);
        double turnMagnitude = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND);

        Vector translation = new Vector(translateX, translateY);   

        Vector topLeftRotation = new Vector(Drivetrain.DT_WIDTH, Drivetrain.DT_LENGTH);
        Vector topRightRotation = new Vector(Drivetrain.DT_LENGTH, -Drivetrain.DT_WIDTH);
        Vector backLeftRotation = new Vector(-Drivetrain.DT_WIDTH, Drivetrain.DT_LENGTH);
        Vector backRightRotation = new Vector(-Drivetrain.DT_WIDTH, -Drivetrain.DT_LENGTH);

        topLeftRotation.scale(turnMagnitude / ROOT);
        topRightRotation.scale(turnMagnitude / ROOT);
        backLeftRotation.scale(turnMagnitude / ROOT);
        backRightRotation.scale(turnMagnitude / ROOT);
        
        Vector sumTopLeft = Vector.add(topLeftRotation, translation).scale(OUTPUT_MULTIPLIER);
        Vector sumTopRight = Vector.add(topRightRotation, translation).scale(OUTPUT_MULTIPLIER);
        Vector sumBackLeft = Vector.add(backLeftRotation, translation).scale(OUTPUT_MULTIPLIER);
        Vector sumBackRight = Vector.add(backRightRotation, translation).scale(OUTPUT_MULTIPLIER);
        
        // Scale down the vectors so that the largest magnitude is at the maximum speed
        double largestMag = max4(sumTopRight.getMagnitude(), sumTopLeft.getMagnitude(), sumBackRight.getMagnitude(), sumBackLeft.getMagnitude());
		
		if(largestMag < 1) 
			largestMag = 1;

		sumTopRight.scale(1 / largestMag);
		sumTopLeft.scale(1 / largestMag);
		sumBackRight.scale(1 / largestMag);
		sumBackLeft.scale(1 / largestMag);

        Drivetrain.getInstance().setDrivetrain(sumTopRight, sumTopLeft, sumBackRight, sumBackLeft);
    }

    public static double max4(double a, double b, double c, double d) {
		return Math.max(Math.max(a, b), Math.max(c, d));
	}
}