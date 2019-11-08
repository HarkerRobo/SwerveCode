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
    private static OI instance;

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;
    
    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;

    private XboxGamepad operatorGamepad;
    private XboxGamepad driverGamepad;

    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);
        initBindings();
    }

    public void initBindings() {
        

        //Controller Bindings below

    }

    public XboxGamepad getDriverGamepad() {
        return driverGamepad;
    }

    public XboxGamepad getOperatorGamepad() {
        return operatorGamepad;
    }
    
    public static OI getInstance() {
        if(instance == null)
            instance = new OI();
        return instance;
    }
}