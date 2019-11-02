package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;

    private static XboxController driverGamepad;
    private static XboxController operatorGamepad;

    public OI() {
        driverGamepad = new XboxController(DRIVER_PORT);
        operatorGamepad = new XboxController(OPERATOR_PORT);

        initBindings();
    }

    public static void initBindings() {
    }

    public XboxController getDriverGamepad() {
        return driverGamepad;
    }

    public XboxController getOperatorGamepad() {
        return operatorGamepad;
    }
}