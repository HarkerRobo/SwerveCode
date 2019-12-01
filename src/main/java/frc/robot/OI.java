package frc.robot;

import frc.robot.commands.SwerveDriveWithMotionProfile;
import frc.robot.commands.ToggleFieldSensitivity;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.CallMethodCommand;
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
    
    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;

    private XboxGamepad operatorGamepad;
    private XboxGamepad driverGamepad;

    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);
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
            new Waypoint(2, 1, 0),
            new Waypoint(2.5, 2, 0)
        };

        int timeDur = 10; //ms between each segment

        // OI.getInstance().getDriverGamepad().getUpDPadButton().whenPressed(
        //         new SwerveDriveWithMotionProfile(points, timeDur)
        // );
        driverGamepad.getButtonY().whenPressed(new SwerveDriveWithMotionProfile(forward, timeDur));
        driverGamepad.getButtonA().whenPressed(new SwerveDriveWithMotionProfile(rightAndUp, timeDur));
        driverGamepad.getButtonBumperRight().whenPressed(new ToggleFieldSensitivity());
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