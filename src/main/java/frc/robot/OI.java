package frc.robot;

import harkerrobolib.wrappers.XboxGamepad;

/**
 * Contains controllers (Xbox or DDR) and their button bindings
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @since 11/1/19
 */
public class OI {

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;

    private static XboxGamepad driverGamepad;
    private static XboxGamepad operatorGamepad;

    public static void initBindings() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);

        SmartDashboard.putBoolean("MemeMode enabled?", memeMode());
        operatorGamepad.getButtonA().whenPressed(new ConditionalCommand(() -> memeMode(), new MemeMode(10)));
    }

    public static XboxGamepad getDriverGamepad() {
        return driverGamepad;
    }

    public static XboxGamepad getOperatorGamepad() {
        return operatorGamepad;
    }

    private boolean memeMode() {
        return operatorGamepad.getLeftBumper().getButtonBumperLeftState() && operatorGamepad.RightBumper().getButtonBumperRightState();
    }
}