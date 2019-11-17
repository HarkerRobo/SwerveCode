package frc.robot;

import frc.robot.commands.SwerveDriveWithMotionProfile;
import harkerrobolib.wrappers.XboxGamepad;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;
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

    Waypoint[] points = new Waypoint[] {
        new Waypoint(-4, -1, Pathfinder.d2r(0)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
        new Waypoint(-3, -1, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
        new Waypoint(-2, -1, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
    };

    private XboxGamepad operatorGamepad;
    private XboxGamepad driverGamepad;

    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);
        initBindings();
    }

    public void initBindings() {
        OI.getInstance().getDriverGamepad().getButtonY().whenPressed(new SwerveDriveWithMotionProfile(points));
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