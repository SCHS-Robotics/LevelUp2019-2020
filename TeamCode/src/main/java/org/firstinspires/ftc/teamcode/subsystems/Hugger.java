package org.firstinspires.ftc.teamcode.subsystems;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.misc.Toggle;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;

/**
 * The block attractor subsystem.
 *
 * @author Jack Kinney
 * @author Ella Player
 */
public class Hugger extends SubSystem {
    //A toggle to enable and disable the skystone attractor.
    private Toggle hugLeftToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    private Toggle hugRightToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    private Toggle hugTopLeftToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    private Toggle hugTopRightToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    //The servos used in the skystone attractor subsystem.
    private Servo leftHugger, leftTopHugger, rightHugger, rightTopHugger;
    //The customizable gamepad used to control the subsystem.
    private CustomizableGamepad gamepad;

    /**
     * A constructor for the skystone attractor subsystem.
     *
     * @param robot The robot using this subsystem.
     * @param huggerServoConfigLeft The config name of the left servo.
     * @param huggerServoConfigRight The config name of the right servo.
     * @param hugger2ServoConfigLeft The config name of the top left servo.
     * @param hugger2ServoConfigRight The config name of the top right servo.
     */
    public Hugger(@NotNull Robot robot, String huggerServoConfigLeft, String huggerServoConfigRight, String hugger2ServoConfigLeft, String hugger2ServoConfigRight) {
        super(robot);
        //Hardwaremaps all the servos.
        leftHugger = robot.hardwareMap.servo.get(huggerServoConfigLeft);
        rightHugger = robot.hardwareMap.servo.get(huggerServoConfigRight);
        leftTopHugger = robot.hardwareMap.servo.get(hugger2ServoConfigLeft);
        rightTopHugger = robot.hardwareMap.servo.get(hugger2ServoConfigRight);

        //Reverses the direction of the left servo.
        leftHugger.setDirection(Servo.Direction.REVERSE);

        //Creates a new customizable gamepad.
        gamepad = new CustomizableGamepad(robot);

        //Notifies the subsystem to use the HAL9001 config system. This variable is defined in the subsystem superclass.
        usesConfig = true;
    }

    @Override
    public void init() {
        //Resets all the servos.
        reset();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        //If the subsystem uses config, pull the controls for the subsystem.
        if (usesConfig) {
            gamepad = robot.pullControls(this);
        }
    }

    @Override
    public void handle() {
        //Update the toggle with the current value of the button.
        hugLeftToggle.updateToggle(gamepad.getBooleanInput("HuggerLeft"));

        //If the toggle is activated, set the servos to the active position, otherwise reset them.
        if (hugLeftToggle.getCurrentState()) {
            hugLeft();
        }
        else {
            resetLeft();
        }

        hugRightToggle.updateToggle(gamepad.getBooleanInput("HuggerRight"));

        //If the toggle is activated, set the servos to the active position, otherwise reset them.
        if (hugRightToggle.getCurrentState()) {
            hugRight();
        }
        else {
            resetRight();
        }

        hugTopLeftToggle.updateToggle(gamepad.getBooleanInput("HuggerTopLeft"));

        //If the toggle is activated, set the servos to the active position, otherwise reset them.
        if (hugTopLeftToggle.getCurrentState()) {
            hugTopLeft();
        }
        else {
            resetTopLeft();
        }

        hugTopRightToggle.updateToggle(gamepad.getBooleanInput("HuggerTopRight"));

        //If the toggle is activated, set the servos to the active position, otherwise reset them.
        if (hugTopRightToggle.getCurrentState()) {
            hugTopRight();
        }
        else {
            resetTopRight();
        }
    }

    @Override
    public void stop() {
        //Reset the servos.
        reset();
    }

    /**
     * Activate the block attractor.
     */
    public void hug() {
        hugLeft();
        hugRight();
    }

    /**
     * Resets the block attractor.
     */
    public void reset() {
        resetLeft();
        resetRight();
        resetTopLeft();
        resetTopRight();
    }

    /**
     * Activates the left side of the block attractor.
     */
    public void hugLeft() {
        setHuggerPosLeft(0.6);
    }

    /**
     * Resets the left side of the block attractor.
     */
    public void resetLeft() {
        setHuggerPosLeft(0);
    }

    /**
     * Sets the left block attractor to the specified position.
     *
     * @param pos The position to set the servo to between 0 and 1.
     */
    public void setHuggerPosLeft(double pos) {
        leftHugger.setPosition(Range.clip(pos,0,1));
    }

    /**
     * Activates the top left part of the block attractor.
     */
    public void hugTopLeft() {
        setTopHuggerPosLeft(1);
    }

    /**
     * Resets the top left part of the block attractor.
     */
    public void resetTopLeft() {
        setTopHuggerPosLeft(0);
    }

    /**
     * Sets the top left part of the block attractor to the specified position.
     *
     * @param pos The position to set the servo to between 0 and 1.
     */
    public void setTopHuggerPosLeft(double pos) {
        leftTopHugger.setPosition(Range.clip(pos,0,1));
    }

    /**
     * Activates the right part of the block attractor.
     */
    public void hugRight() {
        setHuggerPosRight(1);
    }

    /**
     * Activates the top right part of the block attractor.
     */
    public void hugTopRight() {
        setTopHuggerPosRight(0);
    }

    /**
     * Resets the right part of the block attractor.
     */
    public void resetRight() {
        setHuggerPosRight(0);
    }

    /**
     * Resets the top right part of the block attractor.
     */
    public void resetTopRight() {
        setTopHuggerPosRight(1);
    }

    /**
     * Sets the right part of the block attractor to the specified position.
     *
     * @param pos The position to servo to between 0 and 1.
     */
    public void setHuggerPosRight(double pos) {
        rightHugger.setPosition(Range.clip(pos,0,1));
    }

    /**
     * Sets the top right part of the block attractor to the specified position.
     *
     * @param pos The position to servo to between 0 and 1.
     */
    public void setTopHuggerPosRight(double pos) {
        rightTopHugger.setPosition(Range.clip(pos,0,1));
    }

    /**
     * The teleop configuration settings.
     *
     * @return The teleop configuration settings.
     */
    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam("HuggerLeft", Button.BooleanInputs.bool_left_trigger, 2),
                new ConfigParam("HuggerRight", Button.BooleanInputs.bool_right_trigger, 2),
                new ConfigParam("HuggerTopLeft", Button.BooleanInputs.left_bumper, 2),
                new ConfigParam("HuggerTopRight", Button.BooleanInputs.right_bumper, 2),
        };
    }
}
