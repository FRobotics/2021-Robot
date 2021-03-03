package frc.robot.base.input;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;

public class Controller {

    private Joystick joystick;
    private HashMap<Integer, Boolean> buttonsPressed;
    private HashMap<Integer, Boolean> axisPressed;

    public Controller(Joystick joystick) {
        this.joystick = joystick;
        this.buttonsPressed = new HashMap<>();
        this.axisPressed = new HashMap<>();
        
        for (Button button : Button.values()) {
            int id = button.getId();
            buttonsPressed.put(id, false);
        }
    }
    
    public Controller(int port) {
        this(new Joystick(port));
    }

    /**
     * @param button the button you want to specify
     * @return whether the specified button is current being pressed
     */
    public boolean buttonDown(Button button) {
        return joystick.getRawButton(button.getId());
    }

    /**
     * Returns true if the button was "just pressed"
     * 
     * @param button the button you want to specify
     * @return whether the button was pressed but not during the loop before
     */
    public boolean buttonPressed(Button button) {
        return !buttonsPressed.get(button.getId()) && buttonDown(button);
    }

    /**
     * Returns the value of an axis on the controller
     * 
     * @param axis the axis you want to measure
     * @return the value of the axis
     */
    public double getAxis(Axis axis) {
        return joystick.getRawAxis(axis.getId());
    }

    /**
     * Returns whether an axis is currently "pressed" or not
     * @param axis the axis you want to test
     * @return whether the value is > 0.5 or < -0.5
     */
    public boolean axisDown(Axis axis) {
        return Math.abs(joystick.getRawAxis(axis.getId())) > 0.5;
    }

    /**
     * Returns true if an axis was "just pressed"
     * @param axis the axis you want to test
     * @return whether the value is > 0.5 or < -0.5 but not during the loop before
     */
    public boolean axisPressed(Axis axis) {
        return !axisPressed.get(axis.getId()) && axisDown(axis);
    }

    /**
     * Returns the value of a pov on the controller
     * 
     * @param pov the pov you want to measure
     * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
     */
    public int getPov(Pov pov) {
        return joystick.getPOV(pov.getId());
    }

    /**
     * A method that should be run after the main periodic code;
     * makes buttonPressed(Button button) work
     */
    public void postPeriodic() {
        for (Button button : Button.values()) {
            buttonsPressed.put(button.getId(), buttonDown(button));
        }
        for(Axis axis : Axis.values()) {
            axisPressed.put(axis.getId(), axisDown(axis));
        }
    }

}