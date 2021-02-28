package frc.robot.base.util;

import frc.robot.base.device.DoubleSolenoid4150;

import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;

public class Util {

    /**
     * @param solenoid the solenoid
     * @return a supplier of the solenoid state as a string for NTVs
     */
    public static Supplier<Object> solenoidNTV(DoubleSolenoid4150 solenoid) {
        return () -> {
            switch (solenoid.getRaw()) {
                case kForward:
                    return "forward";
                case kReverse:
                    return "reverse";
                case kOff:
                    return "off";
                default:
                    return "???";
            }
        };
    }

    /**
     * Scales a value so the range (deadband, 1) becomes (0, 1) as well as applies a
     * power to smooth joystick control
     */
    public static double adjustInput(double input, double deadBand, int power) {
        double absInput = Math.abs(input);
        double deadBanded = absInput < deadBand ? 0 : (absInput - deadBand) * (1 / (1 - deadBand));
        double smoothed = Math.pow(deadBanded, power);
        return input > 0 ? smoothed : -smoothed;
    }

    @SuppressWarnings("unchecked")
    public static <T> Entry<String, Consumer<Object>> setter(String name, Consumer<T> f) {
        return Map.entry(name, (Object obj) -> {
            try {
                if (obj != null) {
                    f.accept((T) obj);
                }
            } catch (ClassCastException e) {
                DriverStation.reportError("A setter used the wrong type", false);
            }
        });
    }
}
