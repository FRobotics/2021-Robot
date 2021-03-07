package frc.robot.base;

import frc.robot.base.input.Axis;
import frc.robot.base.input.Button;

public class Controls {
    public static class DriveTrain {
        public static Axis DRIVE_FORWARD_BACKWARD = Axis.LEFT_Y;
        public static Axis TURN_LEFT_RIGHT = Axis.RIGHT_X;
        public static Button USE_CLOSED_LOOP = Button.START;
        public static Button DONT_USE_CLOSED_LOOP = Button.BACK;
    }
}
