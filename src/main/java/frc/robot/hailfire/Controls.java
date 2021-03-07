package frc.robot.hailfire;

import frc.robot.base.input.Axis;
import frc.robot.base.input.Button;
import frc.robot.base.input.Controller;
import frc.robot.base.input.Pov;

public class Controls {
    
    public static final Controller drive = new Controller(0);
    public static final Controller shooter = new Controller(1);
    public static final Controller aux = new Controller(2);

    // NOTE: these aren't really used anymore
    public static class Climber {
        public static boolean RETRACT() { return aux.buttonDown(Button.START); }
        public static boolean EXTEND() { return aux.buttonDown(Button.BACK); }
        public static boolean SPIN_WINCH() { return aux.buttonDown(Button.RIGHT_BUMPER); }
    }

    // NOTE: there are more controls in base/Controls.java
    public static class DriveTrain {
        public static boolean TURN_RIGHT() { return drive.buttonDown(Button.B); }
        public static boolean TURN_LEFT() { return drive.buttonDown(Button.X); }
        public static boolean LOW_GEAR() { return drive.buttonPressed(Button.LEFT_BUMPER); }
        public static boolean HIGH_GEAR() { return drive.buttonPressed(Button.RIGHT_BUMPER); }
        public static boolean AUTO_SHIFT() { return drive.getPov(Pov.D_PAD) >= 0; }
        public static boolean AUTO_AIM() { return drive.buttonDown(Button.Y); }
        public static boolean TOGGLE_REVERSE() { return drive.buttonPressed(Button.A); }
    }

    public static class Intake {
        public static boolean ARM_UP() { return aux.buttonPressed(Button.Y); }
        public static boolean ARM_DOWN() { return aux.buttonPressed(Button.A); }
        public static boolean SPIN_FORWARD() { return aux.buttonDown(Button.B); }
        public static boolean SPIN_BACKWARD() { return aux.buttonDown(Button.X); }
    }

    public static class Shooter {
        public static boolean SHOOT() { return shooter.buttonDown(Button.RIGHT_BUMPER); }
        public static boolean MANUAL_CAROUSEL_LEFT() { return shooter.getAxis(Axis.LEFT_X) < -0.5; }
        public static boolean MANUAL_CAROUSEL_RIGHT() { return shooter.getAxis(Axis.LEFT_X) > 0.5; }
        public static boolean AUTO_CAROUSEL_LEFT() { return shooter.axisDown(Axis.LEFT_TRIGGER); }
        public static boolean AUTO_CAROUSEL_RIGHT() { return shooter.axisDown(Axis.RIGHT_TRIGGER); }
        public static boolean CAROUSEL_UP() { return shooter.buttonDown(Button.A); }
        public static boolean CAROUSEL_DOWN() { return shooter.buttonDown(Button.Y); }
        public static boolean TOGGLE_LIGHTS() { return shooter.buttonPressed(Button.LEFT_BUMPER); }
        public static boolean TEST_PITCH() { return shooter.buttonPressed(Button.X); }
    }
}
