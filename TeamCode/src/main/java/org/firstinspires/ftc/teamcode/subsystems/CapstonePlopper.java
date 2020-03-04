package org.firstinspires.ftc.teamcode.subsystems;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The capstone release subsystem.
 *
 * @author Jack Kinney
 * @author Ella Player
 */
public class CapstonePlopper extends SubSystem {

    //The name of the capstone release button.
    private static final String PLOP = "plop";
    //The servo used to release the capstone.
    private Servo plop;
    //The customizable gamepad used to control the capstone release system.
    private CustomizableGamepad gamepad;

    /**
     * A constructor for the capstone release subsystem.
     *
     * @param robot The robot using this subsystem.
     * @param plopperConfig The config name for the capstone release servo.
     * @param plopButton The button used to control the capstone release servo.
     */
    public CapstonePlopper(Robot robot, String plopperConfig, Button plopButton) {
        super(robot);
        //Hardwaremaps the capstone release servo.
        plop = robot.hardwareMap.servo.get(plopperConfig);
        //Creates the customizable gamepad.
        gamepad = new CustomizableGamepad(robot);
        //Adds the capstone release button to the gamepad.
        gamepad.addButton(PLOP, plopButton);
    }

    /**
     * A constructor for the capstone release subsystem. This uses the HAL9001 config system.
     *
     * @param robot The robot using this subsystem.
     * @param plopperConfig The configuration name of the capstone release servo.
     */
    public CapstonePlopper(Robot robot, String plopperConfig) {
        super(robot);
        //Hardwaremaps the capstone release servo.
        plop = robot.hardwareMap.servo.get(plopperConfig);
        //Creates the customizable gamepad.
        gamepad = new CustomizableGamepad(robot);

        //Tells the subsystem to use the HAL9001 config system. This variable is present in the subsystem superclass.
        usesConfig = true;
    }

    @Override
    public void init() {
        //Resets the capstone release system.
        resetPlop();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        //If the subsystem uses the config system, pull the controls for it from the config system.
        if(usesConfig) {
            gamepad = robot.pullControls(this);
        }
    }

    @Override
    public void handle() {
        //If the release button is pressed, release the capstone. Otherwise reset the release system.
        if(gamepad.getBooleanInput(PLOP)) {
            plop();
        }
        else {
            resetPlop();
        }
    }

    @Override
    public void stop() {

    }

    /**
     * Sets the capstone release servo to a certain position.
     *
     * @param position The position to set the servo to.
     */
    public void setPlop(double position) {
        plop.setPosition(position);
    }

    /**
     * Release the capstone.
     */
    public void plop() {
        setPlop(0);
    }

    /**
     * Reset the capstone release system.
     */
    public void resetPlop() {
        setPlop(0.4);
    }

    /**
     * Activate the capstone release system for a certain amount of time.
     *
     * @param time The time to release in milliseconds.
     */
    public void plopTime(long time) {
        plop();
        waitTime(time);
        resetPlop();
    }

    /**
     * The teleop config for this subsystem.
     *
     * @return The teleop config for this subsystem.
     */
    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam(PLOP, Button.BooleanInputs.a)
        };
    }
}
