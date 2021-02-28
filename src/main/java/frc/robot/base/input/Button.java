package frc.robot.base.input;

// enum
public enum Button {
    A(1), B(2), X(3), Y(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), BACK(7), START(8);

    private int ID;

    Button(int ID) {
        this.ID = ID;
    }

    public int getId() {
        return ID;
    }

}
