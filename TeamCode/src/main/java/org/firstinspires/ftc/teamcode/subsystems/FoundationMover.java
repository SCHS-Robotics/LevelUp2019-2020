package org.firstinspires.ftc.teamcode.subsystems;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The foundation mover subsystem.
 *
 * @author Jack Kinney
 * @author Tristan Tran
 * @author Ella Player
 */
public class FoundationMover extends SubSystem {

    //The name of the button used to activate the foundation mover.
    private static final String LATCH = "latch";
    //The servos used to latch on the the foundation.
    private Servo latch, latch2;
    //The customizable gamepad used to control the foundarion mover.
    private CustomizableGamepad gamepad;

    /**
     * A constructor for the foundation mover subsystem.
     *
     * @param robot The robot using this subsystem.
     * @param latchConfigLeft The configuration name of the left latch servo.
     * @param latchConfigRight The configuration name of the right latch servo.
     * @param plopButton The button used to activate the foundation mover.
     */
    public FoundationMover(Robot robot, String latchConfigLeft, String latchConfigRight, Button plopButton) {
        super(robot);
        //Hardwaremaps the latch servos.
        latch = robot.hardwareMap.servo.get(latchConfigLeft);
        latch2 = robot.hardwareMap.servo.get(latchConfigRight);

        //Sets the correct directions for the two latches.
        latch.setDirection(Servo.Direction.REVERSE);
        latch2.setDirection(Servo.Direction.FORWARD);

        //Create the customizable gamepad and add the latch button to the gamepad.
        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(LATCH, plopButton);
    }

    /**
     * A constructor for the foundation mover subsystem. This uses the HAL9001 config system.
     *
     * @param robot The robot using this subsystem.
     * @param latchConfigLeft The configuration name of the left latch servo.
     * @param latchConfigRight The configuration name of the right latch servo.
     */
    public FoundationMover(Robot robot, String latchConfigLeft, String latchConfigRight) {
        super(robot);
        //Hardwaremaps the latch servos.
        latch = robot.hardwareMap.servo.get(latchConfigLeft);
        latch2 = robot.hardwareMap.servo.get(latchConfigRight);

        //Sets the correct directions for the two latches.
        latch.setDirection(Servo.Direction.REVERSE);
        latch2.setDirection(Servo.Direction.FORWARD);

        //Create the customizable gamepad.
        gamepad = new CustomizableGamepad(robot);
        //Notifies the subsystem to use the HAL9001 config system.
        usesConfig = true;
    }

    @Override
    public void init() {
        resetLatch();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        //If the subsystem uses the config system, pull the configured controls for that subsystem.
        if(usesConfig) {
            gamepad = robot.pullControls(this);
        }
    }

    @Override
    public void handle() {
        //If the latch button is pressed, latch onto the foundation, otherwise reset the latches.
        if(gamepad.getBooleanInput(LATCH)) {
            latch();
        }
        else {
            resetLatch();
        }
    }

    @Override
    public void stop() {
        //Resets the latches.
        resetLatch();
    }

    /**
     * Sets the latches to the given position.
     *
     * @param position The position between 0 and 1.
     */
    public void setLatch(double position) {
        latch.setPosition(position);
        latch2.setPosition(position/2);
    }

    /**
     * Latches on to the foundation.
     */
    public void latch() {
        setLatch(0);
    }

    /**
     * Resets the latches.
     */
    public void resetLatch() {
        setLatch(1);
    }

    /**
     * Activates the latches for a certain amount of time.
     *
     * @param time The amount of time in milliseconds to latch for.
     */
    public void latchTime(long time) {
        latch();
        waitTime(time);
        resetLatch();
    }

    /**
     * The teleop config settings.
     *
     * @return The teleop config settings.
     */
    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam(LATCH, Button.BooleanInputs.b)
        };
    }
}