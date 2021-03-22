package frc.robot.base.util;

import java.util.List;

import frc.robot.hailfire.Hailfire;

public class CommandList {
    private List<Command> commands;
    private boolean finished;
    private int step = 0;
    private Boolean initialize = true;

    public boolean Finished() {
        return finished;
    }
    public CommandList(List<Command> commands) {
        this.commands = commands;
        step = 0;
    }
    public void Execute(Hailfire hailfire) {
        if (step >= commands.size()) {
            finished = true;
            initialize = true;
            return;
        }
        Command currentCommand = commands.get(step);
        switch(currentCommand.type) {
        case DRIVE:
            double currentDist = hailfire.driveTrain.getLeftDistance();
            if (initialize) {
                currentCommand.arguments[3] = currentDist + currentCommand.arguments[0];
                initialize = false;
            }
                
            hailfire.driveTrain.autoDriveForward(currentCommand.arguments[3] - currentDist);

            if (currentCommand.arguments[3] - currentDist <= currentCommand.arguments[1]) {
                step++;
                initialize = true;
            }
                
            break;
        case SLEEP:
            // USAGE: SLEEP ( milliseconds )
            if (initialize) {
                currentCommand.arguments[3] = System.currentTimeMillis();
                initialize = false;
            }
            if (System.currentTimeMillis() >= currentCommand.arguments[3] + currentCommand.arguments[0]) {
                initialize = true;
                step++;
            }
            break;
        case TURN:
            // USAGE: TURN ( amtInDegrees, threshold )
            double currentAngle = hailfire.driveTrain.gyro.getAngle();
            if (initialize) {
                currentCommand.arguments[3] = hailfire.driveTrain.gyro.getAngle();
                initialize = false;
            }
            double targetAngle = currentCommand.arguments[3] - currentCommand.arguments[0];
            System.out.println(currentCommand.arguments[3] - currentCommand.arguments[0]);
            hailfire.driveTrain.turnToAngle(targetAngle);
            if (Math.abs(currentAngle - targetAngle) <= currentCommand.arguments[1]) {                    initialize = true;
                    step++;
            }
            break;
        case PIXYGOTO:
            initialize = false;
            double pixyOut = hailfire.driveTrain.ReadPixy();
                
            if (pixyOut == -1000.0d) {
                initialize = true;
                step = (int)currentCommand.arguments[1];
            } else if (pixyOut < currentCommand.arguments[0]) {
                initialize = true;
                step = (int)currentCommand.arguments[2];
            }else if (pixyOut >= currentCommand.arguments[0]) {
                initialize = true;
                step = (int)currentCommand.arguments[3];
            }
        case BALL_PICKUP:        
            break;
        default:
            break;
        }
    }
}