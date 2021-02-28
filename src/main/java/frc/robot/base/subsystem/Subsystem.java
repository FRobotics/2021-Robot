package frc.robot.base.subsystem;

import frc.robot.base.RobotMode;
import frc.robot.base.action.Action;
import frc.robot.base.action.ActionHandler;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

/*
 * NOTE: for the devices in the subsystem it would probably make sense to make them registrable
 * and have default states so you don't have to set everything in each mode
 * (this used to be a todo but it's not important)
 */

/**
 * An action handler meant to deal with a group of devices that work together
 */
public abstract class Subsystem extends ActionHandler {

    private final Action STOP = new Action(this::stop);
    private final Action CONTROL = new Action(this::control);

    public final String name;

    /**
     * Creates a new subsystem
     * @param name the name of the subsystem
     */
    public Subsystem(String name) {
        this.name = name;
        this.startActionAndSetDefault(STOP);
    }

    public void stop() {}
    public void control() {}

    /**
     * Use this method to specify what values you want to put on the dashboard;
     * The keys are the names of the entries and the suppliers are functions that return the values to set the entries to
     */
    public Map<String, Supplier<Object>> NTSets() {
        return Map.of();
    }

    /**
     * Use this method to specify what values you want to get from the dashboard;
     * The keys are the names of the entries and the consumers are functions that use the values
     */
    public Map<String, Consumer<Object>> NTGets() {
        return Map.of();
    }

    /**
     * called during the beginning of each mode
     *
     * @param mode the mode the robot is in
     */
    public void onInit(RobotMode mode) {
        this.clearActionQueue();
        switch (mode) {
            case AUTONOMOUS:
            case DISABLED:
                startActionAndSetDefault(STOP);
                break;
            case TEST:
            case TELEOP:
                startActionAndSetDefault(CONTROL);
                break;
        }
    }
}
