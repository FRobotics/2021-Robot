package frc.robot.base.action;

/**
 * A function meant to be periodically called for a specified amount of time
 */
public class TimedAction extends SetupAction {

    private long startTime;

    public TimedAction(ActionFunc action, int length) {
        this.func = action;
        this.setupAction = () -> startTime = System.currentTimeMillis();
        this.finishCondition = () -> (System.currentTimeMillis() - startTime) > length;
    }

    public TimedAction(int length) {
        this(() -> {}, length);
    }
}
