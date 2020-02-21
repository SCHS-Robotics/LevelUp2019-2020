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

public class Hugger extends SubSystem {
    private Toggle hugToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    private Servo leftHugger, leftTopHugger, rightHugger, rightTopHugger;
    private CustomizableGamepad gamepad;
    public Hugger(@NotNull Robot robot, String huggerServoConfigLeft, String huggerServoConfigRight, String hugger2ServoConfigLeft, String hugger2ServoConfigRight) {
        super(robot);
        leftHugger = robot.hardwareMap.servo.get(huggerServoConfigLeft);
        rightHugger = robot.hardwareMap.servo.get(huggerServoConfigRight);
        leftTopHugger = robot.hardwareMap.servo.get(hugger2ServoConfigLeft);
        rightTopHugger = robot.hardwareMap.servo.get(hugger2ServoConfigRight);

        leftHugger.setDirection(Servo.Direction.REVERSE);

        gamepad = new CustomizableGamepad(robot);
        usesConfig = true;
    }

    @Override
    public void init() {
        reset();
        hugTopLeft();
        hugTopRight();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        if (usesConfig) {
            gamepad = robot.pullControls(this);
        }
    }

    @Override
    public void handle() {
        hugToggle.updateToggle(gamepad.getBooleanInput("Hugger"));
        if (hugToggle.getCurrentState()) {
            hug();
        }
        else {
            reset();
        }
    }

    @Override
    public void stop() {

    }

    public void hug() {
        hugLeft();
        hugRight();
    }

    public void hugTime(long timeMillis) {
        hug();
        waitTime(timeMillis);
        reset();
    }

    public void reset() {
        resetLeft();
        resetRight();
        resetTopLeft();
        resetTopRight();
    }

    public void hugLeft() {
        setHuggerPosLeft(0.6);
    }

    public void hugLeftTime(long timeMillis) {
        hugLeft();
        waitTime(timeMillis);
        resetLeft();
    }

    public void resetLeft() {
        setHuggerPosLeft(0);
    }

    public void setHuggerPosLeft(double pos) {
        leftHugger.setPosition(Range.clip(pos,0,1));
    }

    public void hugTopLeft() {
        setTopHuggerPosLeft(1);
    }

    public void resetTopLeft() {
        setTopHuggerPosLeft(0);
    }

    public void setTopHuggerPosLeft(double pos) {
        leftTopHugger.setPosition(Range.clip(pos,0,1));
    }

    public void hugRight() {
        setHuggerPosRight(1);
    }

    public void hugTopRight() {
        setTopHuggerPosRight(1);
    }

    public void hugRightTime(long timeMillis) {
        hugRight();
        waitTime(timeMillis);
        resetRight();
    }

    public void resetRight() {
        setHuggerPosRight(0.5);
    }

    public void resetTopRight() {
        setTopHuggerPosRight(0);
    }

    public void setHuggerPosRight(double pos) {
        rightHugger.setPosition(Range.clip(pos,0,1));
    }

    public void setTopHuggerPosRight(double pos) {
        rightTopHugger.setPosition(Range.clip(pos,0,1));
    }
    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam("Hugger", Button.BooleanInputs.dpad_down),
        };
    }
}
