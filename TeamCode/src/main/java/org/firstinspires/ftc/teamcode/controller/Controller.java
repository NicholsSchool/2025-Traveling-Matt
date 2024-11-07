package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A Controller
 */
public class Controller {
    private final Gamepad gamepad;
    public final Button leftBumper;
    public final Button rightBumper;
    public final Button dpadUp;
    public final Button dpadDown;
    public final Button dpadLeft;
    public final Button dpadRight;
    public final Button x;
    public final Button circle;
    public final Button square;
    public final Button triangle;
    public final Button share;
    public final Button options;
    public final Button leftStickButton;
    public final Button rightStickButton;
    public final Axis leftTrigger;
    public final Axis rightTrigger;
    public final Joystick leftStick;
    public final Joystick rightStick;

    /**
     * Instantiates the Controller
     *
     * @param gamepad the gamepad to use
     */
    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;

        leftBumper = new Button();
        rightBumper = new Button();
        dpadUp = new Button();
        dpadDown = new Button();
        dpadLeft = new Button();
        dpadRight = new Button();
        x = new Button();
        circle = new Button();
        square = new Button();
        triangle = new Button();
        share = new Button();
        options = new Button();
        leftStickButton = new Button();
        rightStickButton = new Button();

        leftTrigger = new Axis();
        rightTrigger = new Axis();

        leftStick = new Joystick();
        rightStick = new Joystick();
    }

    /**
     * Updates all Buttons, Axes, and Joysticks
     */
    public void update() {
        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);
        dpadUp.update(gamepad.dpad_up);
        dpadDown.update(gamepad.dpad_down);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        x.update(gamepad.a);
        circle.update(gamepad.b);
        square.update(gamepad.x);
        triangle.update(gamepad.y);
        share.update(gamepad.back);
        options.update(gamepad.start);
        leftStickButton.update(gamepad.left_stick_button);
        rightStickButton.update(gamepad.right_stick_button);

        leftTrigger.update(gamepad.left_trigger);
        rightTrigger.update(gamepad.right_trigger);

        leftStick.update(gamepad.left_stick_x, -gamepad.left_stick_y);
        rightStick.update(gamepad.right_stick_x, -gamepad.right_stick_y);
    }
}