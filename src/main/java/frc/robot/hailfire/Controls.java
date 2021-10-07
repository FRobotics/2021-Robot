package frc.robot.hailfire;

import frc.robot.base.input.Axis;
import frc.robot.base.input.Button;
import frc.robot.base.input.Controller;
import frc.robot.base.input.Pov;

public class Controls {
    
    public static final Controller drive = new Controller(0);
    public static final Controller aux = new Controller(1);

    // NOTE: these aren't really used anymore
    public static class Climber {
        public static boolean RETRACT() { return aux.buttonDown(Button.LEFT_BUMPER); }
        public static boolean EXTEND() { return aux.buttonDown(Button.RIGHT_BUMPER); }
        public static boolean SPIN_WINCH() { return aux.axisDown(Axis.LEFT_TRIGGER); }
        public static boolean SPIN_WINCH_BACK() { return aux.axisDown(Axis.RIGHT_TRIGGER); }
    }

    // NOTE: there are more controls in base/Controls.java
    public static class DriveTrain {
        public static boolean TURN_RIGHT() { return drive.getPov(Pov.D_PAD) == 90; }
        public static boolean TURN_LEFT() { return drive.getPov(Pov.D_PAD) == 270; }
        public static boolean LOW_GEAR() { return drive.buttonPressed(Button.RIGHT_BUMPER); }
        public static boolean HIGH_GEAR() { return drive.buttonPressed(Button.LEFT_BUMPER); }
        public static boolean AUTO_SHIFT() { return drive.buttonPressed(Button.START); }
        public static boolean AUTO_AIM() { return drive.buttonDown(Button.B); }
        public static boolean TOGGLE_REVERSE() { return drive.buttonPressed(Button.A); }
    }

    public static class Intake {
        public static boolean ARM_UP() { return aux.getPov(Pov.D_PAD) == 0; }
        public static boolean ARM_DOWN() { return aux.getPov(Pov.D_PAD) == 180; }
        public static boolean SPIN_FORWARD() { return aux.getPov(Pov.D_PAD) == 270; }
        public static boolean SPIN_BACKWARD() { return aux.getPov(Pov.D_PAD) == 90; }
    }

    public static class Shooter {
        public static boolean SHOOT() { return drive.axisDown(Axis.RIGHT_TRIGGER); }
        public static boolean MANUAL_CAROUSEL_LEFT() { return false; }
        public static boolean MANUAL_CAROUSEL_RIGHT() { return false; }
        public static boolean AUTO_CAROUSEL_LEFT() {  return aux.buttonPressed(Button.X); }
        public static boolean AUTO_CAROUSEL_RIGHT() {  return aux.buttonPressed(Button.B); }
        public static boolean CAROUSEL_UP() { return aux.buttonDown(Button.Y); }
        public static boolean CAROUSEL_DOWN() { return aux.buttonDown(Button.A); }
        public static boolean TOGGLE_LIGHTS() { return aux.buttonPressed(Button.START); }
        public static boolean TEST_PITCH() { return false; }


    }
}
