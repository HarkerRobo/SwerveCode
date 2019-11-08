package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SwervePercentOutput extends IndefiniteCommand {

    public SwervePercentOutput() {
        requires(Drivetrain.getInstance());
    }

    @Override
    protected void execute() {
        double translateX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND);
        double translateY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND);
        double angle = OI.getInstance().getDriverGamepad().getRightX();

        

    }
}