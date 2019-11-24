package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ToggleFieldSensitivity extends InstantCommand {
    private final static int TIMEOUT = 500;
    private static long lastExecututionTime;

    public ToggleFieldSensitivity() {
        // this is a constructor
        lastExecututionTime = System.currentTimeMillis();
    }

    public void initialize() {
        if((int)(System.currentTimeMillis() - lastExecututionTime) > TIMEOUT) {
            Drivetrain.getInstance().toggleFieldSensitivity();
            lastExecututionTime = System.currentTimeMillis();
        }
    }
}