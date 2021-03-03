package frc.robot.base.action;

import java.util.List;

public abstract class ActionHandler {

    private Action action;
    private Action defaultAction;

    private int queuePos;
    private List<? extends Action> actionQueue;

    private boolean finished = true;

    public void startAction(Action action) {
        finished = false;
        this.action = action;
        if(this.action instanceof SetupAction) {
            ((SetupAction)this.action).setupAction.run();
        }
    }

    public void startActionAndSetDefault(Action action) {
        this.startAction(action);
        this.defaultAction = action;
        finished = true;
    }

    public void periodic() {
        if (
                action.isFinished()
        ) {
            if(actionQueue != null) {
                if (queuePos == actionQueue.size()) {
                    // if there's no more states in the queue
                    actionQueue = null;
                    queuePos = 0;
                    this.startAction(defaultAction);
                    finished = true;
                } else {
                    // if there's another state in the queue
                    this.startAction(actionQueue.get(queuePos++));
                }
            } else {
                this.startAction(defaultAction);
                this.finished = true;
            }
        }
        this.action.func.run();
    }

    public void clearActionQueue() {
        this.queuePos = 0;
        this.actionQueue = null;
    }

    public Action getAction() {
        return action;
    }
    
    public Action getDefaultAction() {
        return this.defaultAction;
    }

    public void startActionQueue(List<? extends Action> actionQueue) {
        if (actionQueue.size() == 0) return;
        this.queuePos = 1;
        this.startAction(actionQueue.get(0));
        this.actionQueue = actionQueue;
    }

    public boolean isFinished() {
        return finished;
    }
}
