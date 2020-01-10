package frc.robot;

import frc.robot.util.HSDDRPad;
import harkerrobolib.wrappers.XboxGamepad;
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
        Waypoint[] forward = new Waypoint[] {
            new Waypoint(0, 0, 0),
            new Waypoint(2, 0, 0)
        };
        Waypoint[] rightAndUp = new Waypoint[] {
            new Waypoint(0, 0, 0),
            // new Waypoint(2, 0, 0),
            new Waypoint(1, 0, 0),
            new Waypoint(2, 1, Math.PI / 4),
            new Waypoint(2.5, 2, 0) 
        };
        Waypoint[] backward = new Waypoint[] {
            new Waypoint(0, 0, 0),
            new Waypoint(-2, 0, 0)
        };
        Waypoint[] upAndDiag = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(0, 1.5, Math.PI / 2),
            new Waypoint(0.7, 2.2, Math.PI / 4)
        };
        Waypoint[] semiCircle = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(1, 1, 0),
            new Waypoint(2, 0, Math.PI / -2)
        };

        int timeDur = 10; //ms between each segment

        // OI.getInstance().getDriverGamepad().getUpDPadButton().whenPressed(
        //         new SwerveDriveWithMotionProfile(points, timeDur)
        // );
        
        // driverGamepad.getButtonB().whenPressed(new SwerveDriveWithMotionProfile(semiCircle, timeDur));
        // driverGamepad.getButtonX().whenPressed(new SwerveDriveWithMotionProfile(backward, timeDur));
        // driverGamepad.getButtonA().whenPressed(new SwerveDriveWithMotionProfile(rightAndUp, timeDur));
        // driverGamepad.getButtonY().whenPressed(new SwerveDriveWithMotionProfile(upAndDiag, timeDur));
        // driverGamepad.getButtonBumperRight().whenPressed(new ToggleFieldSensitivity());
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