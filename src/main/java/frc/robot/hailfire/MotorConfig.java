package frc.robot.hailfire;

import frc.robot.base.device.motor.EncoderMotorConfig;

public class MotorConfig {
    public static class DriveTrain {
        public static final EncoderMotorConfig LOW_CONFIG = new EncoderMotorConfig(
                3f/12f, // wheel radius
                4 * 360, // counts per rev
                2.02895, // f
                1.76430, // p
                0.00264, // i
                0.02205, // d
                150 // i zone
        );
        public static final EncoderMotorConfig HIGH_CONFIG = new EncoderMotorConfig(
                3f/12f,
                4 * 360,
                0.5873,
                0.51069,
                0.0012,
                0.00638,
                150
        );
    }

    public static class Shooter {
        public static EncoderMotorConfig CONFIG = new EncoderMotorConfig(
                2048 * 4, // counts per rev
                .0154, // f
                .0060, // p
                .0001, // i
                .0001, // d
                200 // i zone
        );
    }
}
