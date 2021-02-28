package frc.robot.base.input;

// enum
public enum Axis {
    LEFT_X(0), LEFT_Y(1), LEFT_TRIGGER(2), RIGHT_TRIGGER(3), RIGHT_X(4), RIGHT_Y(5);

    private int ID;

    Axis(int ID) {
        this.ID = ID;
    }

    public int getId() {
        return ID;
    }

}
