package frc.robot.base;

import frc.robot.base.action.Action;
import frc.robot.base.action.ActionHandler;

/**
 * The action handler for auto
 */
public class AutoActionHandler extends ActionHandler {
    protected AutoActionHandler() {
        this.startActionAndSetDefault(new Action(() -> {}));
    }
}
