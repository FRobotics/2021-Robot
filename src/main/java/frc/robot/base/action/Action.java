package frc.robot.base.action;

import java.util.function.Supplier;

/**
 * A function meant to be periodically called that ends based on a condition
 */
public class Action {
    protected ActionFunc func;
    protected Supplier<Boolean> finishCondition;

    public Action(ActionFunc func, Supplier<Boolean> finishCondition) {
        this.func = func;
        this.finishCondition = finishCondition;
    }

    public Action(ActionFunc func) {
        this(func, () -> false);
    }

    public Action(Supplier<Boolean> finishCondition) {
        this(() -> {}, finishCondition);
    }

    public boolean isFinished() {
        return finishCondition.get();
    }
}
