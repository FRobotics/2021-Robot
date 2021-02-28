package frc.robot.base.device;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DoubleSolenoid4150 {
    private DoubleSolenoid solenoid;

    public DoubleSolenoid4150(int moduleNumber, int forwardChannel, int backwardChannel) {
        solenoid = new DoubleSolenoid(moduleNumber, forwardChannel, backwardChannel);
    }

    public DoubleSolenoid4150(int forwardChannel, int backwardChannel) {
        solenoid = new DoubleSolenoid(forwardChannel, backwardChannel);
    }

    public boolean extend() {
        if(solenoid.get() != Value.kForward) {
            solenoid.set(Value.kForward);
            return true;
        } else {
            return false;
        }
    }

    public boolean retract() {
        if(solenoid.get() != Value.kReverse) {
            solenoid.set(Value.kReverse);
            return true;
        } else {
            return false;
        }
    }

    public void flip() {
        switch(solenoid.get()) {
            case kOff:
            case kReverse:
                solenoid.set(Value.kForward);
                break;
            case kForward:
                solenoid.set(Value.kReverse);
                break;
        }
    }

    public boolean isExtended() {
        return solenoid.get() == Value.kForward;
    }

    public Value getRaw() {
        return solenoid.get();
    }
}