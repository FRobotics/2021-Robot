package frc.robot.base.util;


public class Command {
    public enum CommandType {
        DRIVE,
        SLEEP,
        TURN,
        PIXYGOTO,
        BALL_PICKUP
    }
    public CommandType type;
    public double[] arguments = {0.0, 0.0, 0.0, 0.0};

    public Command(CommandType type, double arg0, double arg1, double arg2, double arg3) {
        this.type = type;
        arguments[0] = arg0;
        arguments[1] = arg1;
        arguments[2] = arg2;
        arguments[3] = arg3;
    }
}
