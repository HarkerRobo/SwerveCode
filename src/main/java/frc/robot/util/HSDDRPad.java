package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import harkerrobolib.wrappers.HSJoystickButton;

public class HSDDRPad extends Joystick {

    private static final int UP = 1;
    private static final int DOWN = 2;
    private static final int LEFT = 3;
    private static final int RIGHT = 4;
    private static final int X = 7;
    private static final int O = 8;
    private static final int BACK = 9;
    private static final int SELECT = 10;

    private HSJoystickButton upBtn;
    private HSJoystickButton downBtn;
    private HSJoystickButton leftBtn;
    private HSJoystickButton rightBtn;

    private HSJoystickButton xBtn;
    private HSJoystickButton oBtn;
    private HSJoystickButton backBtn;
    private HSJoystickButton selectBtn;

    public HSDDRPad(int port) {
        super(port);

        upBtn = new HSJoystickButton(this, UP);
        downBtn = new HSJoystickButton(this, DOWN);
        leftBtn = new HSJoystickButton(this, LEFT);
        rightBtn = new HSJoystickButton(this, RIGHT);

        xBtn = new HSJoystickButton(this, X);
        oBtn = new HSJoystickButton(this, O);
        backBtn = new HSJoystickButton(this, BACK);
        selectBtn = new HSJoystickButton(this, SELECT);
    }

    public HSJoystickButton getUpBtn() {
        return upBtn;
    }

    public HSJoystickButton getDownBtn() {
        return downBtn;
    }

    public HSJoystickButton getLeftBtn() {
        return leftBtn;
    }

    public HSJoystickButton getRightBtn() {
        return rightBtn;
    }

    public HSJoystickButton getXBtn() {
        return xBtn;
    }

    public HSJoystickButton getOBtn() {
        return oBtn;
    }

    public HSJoystickButton getBackBtn() {
        return backBtn;
    }

    public HSJoystickButton getSelectBtn() {
        return selectBtn;
    }
}