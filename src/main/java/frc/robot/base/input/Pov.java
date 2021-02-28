package frc.robot.base.input;

public enum Pov {
    D_PAD(0);

    private int ID;

    Pov(int ID) {
        this.ID = ID;
    }

    public int getId() {
        return ID;
    }
}