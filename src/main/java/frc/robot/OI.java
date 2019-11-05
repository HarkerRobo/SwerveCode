package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwervePercentOutput;
import harkerrobolib.wrappers.XboxGamepad;

public class OI {

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;

    private static XboxGamepad driverGamepad;
    private static XboxController operatorGamepad;

    public OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxController(OPERATOR_PORT);

        initBindings();
    }

    public static void initBindings() {
        // driverGamepad.getButtonA().whilePressed(new SwerveModulePercentOutput());
        // driverGamepad.getButtonB().whilePressed(new SwerveTurnPercentOutput());
    }

    public static XboxGamepad getDriverGamepad() {
        return driverGamepad;
    }

    public static XboxController getOperatorGamepad() {
        return operatorGamepad;
    }
}