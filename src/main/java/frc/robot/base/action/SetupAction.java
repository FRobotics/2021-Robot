package frc.robot.base.action;

import java.util.function.Supplier;

/**
 * An action that needs to be set up before it runs
 */
public class SetupAction extends Action {

    protected ActionFunc setupAction;

    public SetupAction(ActionFunc setupAction, ActionFunc action, Supplier<Boolean> finishCondition) {
        super(action, finishCondition);
        this.setupAction = setupAction;
    }

    public SetupAction(ActionFunc setupAction, Supplier<Boolean> finishCondition) {
        super(() -> {}, finishCondition);
        this.setupAction = setupAction;
    }

    public SetupAction(ActionFunc setupAction) {
        super(() -> {}, () -> true);
        this.setupAction = setupAction;
    }

    protected SetupAction() {
        super(() -> {}, () -> true);
        this.setupAction = () -> {};
    }
}
