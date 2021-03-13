package frc.robot.hailfire;

public class IDs {

    public static class DriveTrain {
        public static final int LEFT_MOTOR_MASTER = 16;
        public static final int LEFT_MOTOR_FOLLOWER = 17;
        public static final int RIGHT_MOTOR_MASTER = 10;
        public static final int RIGHT_MOTOR_FOLLOWER = 11;

        // public static final int RIGHT_EVO_SHIFTER_FORWARD = 2;
        // public static final int RIGHT_EVO_SHIFTER_REVERSE = 3;
        public static final int LEFT_EVO_SHIFTER_FORWARD = 4;
        public static final int LEFT_EVO_SHIFTER_REVERSE = 5;
}

    public static class Shooter {
        public static final int LEFT_MOTOR = 14;
        public static final int RIGHT_MOTOR = 13;
        public static final int PITCH_MOTOR = 18;
        public static final int CAROUSEL_MOTOR = 19;
    }

    public static class Intake {
        public static final int MOTOR = 12;
        public static final int ARM_FORWARD = 7;
        public static final int ARM_REVERSE = 6;
        public static final int SENSOR = 2;
    }

    public static class Climber {
        public static final int BOTTOM_SOLENOID_FORWARD = 0;
        public static final int BOTTOM_SOLENOID_REVERSE = 1;
        public static final int WINCH_MOTOR = 15;
    }
}
