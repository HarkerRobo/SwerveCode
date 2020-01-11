package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.commands.SwerveDriveWithOdometryProfiling;
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
        driverGamepad.getButtonB().whenPressed(new SwerveDriveWithOdometryProfiling(
            List.of( 
                new Pose2d(0, 0, Rotation2d.fromDegrees(-90)),
                new Pose2d(2, 2, Rotation2d.fromDegrees(-90)),
                new Pose2d(-2, 4, Rotation2d.fromDegrees(-90)),
                new Pose2d(0, 6, Rotation2d.fromDegrees(-90)))));
        // driverGamepad.getButtonX().whenPressed(new SwerveDriveWithMotionProfile(backward, timeDur));
        // driverGamepad.getButtonA().whenPressed(new SwerveDriveWithMotionProfile(rightAndUp, timeDur));
        // driverGamepad.getButtonY().whenPressed(new SwerveDriveWithMotionProfile(upAndDiag, timeDur));
        // driverGamepad.getButtonBumperRight().whenPressed(new ToggleFieldSensitivity());

        driverGamepad.getButtonA().whenPressed(new SwerveDriveWithOdometryProfiling( 
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(2, 0)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0))));
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