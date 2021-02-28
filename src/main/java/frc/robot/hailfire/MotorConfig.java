package frc.robot.hailfire;

import frc.robot.base.device.motor.EncoderMotorConfig;

public class MotorConfig {
    public static class DriveTrain {
        public static final EncoderMotorConfig LOW_CONFIG = new EncoderMotorConfig(
                3f/12f,
                4 * 360,
                2.02895,
                1.76430,
                0.00264,
                0.02205,
                150
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

        /*
         * LOW MULT: 2.2053804347826086956521739130435
         * HIGH MULT: 0.63836956521739130434782608695652
         * OLD CONFIG
         * 0.92,
         * 0.8,
         * 0.0012,
         * 0.01,
         * 150
         */
    }

    public static class Shooter {
        public static EncoderMotorConfig CONFIG = new EncoderMotorConfig(
                2048 * 4,
                .01611,
                .01611,
                .0012,
                .01,
                800
        );
    }
}
