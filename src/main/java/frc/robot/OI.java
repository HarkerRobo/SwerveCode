package frc.robot;

import frc.robot.util.HSDDRPad;
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
    private static OI instance;

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;
    private static final int DDR_PORT = 2;
    
    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;

    private XboxGamepad operatorGamepad;
    private XboxGamepad driverGamepad;
    private HSDDRPad ddrGamepad;

    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);
        ddrGamepad = new HSDDRPad(DDR_PORT);

        initBindings();
    }

    public void initBindings() {
    }

    public XboxGamepad getDriverGamepad() {
        return driverGamepad;
    }

    public XboxGamepad getOperatorGamepad() {
        return operatorGamepad;
    }

    public HSDDRPad getDDRGamepad() {
        return ddrGamepad;
    }
    
    public static OI getInstance() {
        if(instance == null)
            instance = new OI();
        return instance;
    }
}