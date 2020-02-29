package org.firstinspires.ftc.teamcode.subsystems;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.misc.Toggle;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;

public class Stacker extends SubSystem {
    private Toggle clampToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    private Servo clamp;
    private CRServo bladesServo;
    private CustomizableGamepad gamepad;
    public Stacker(@NotNull Robot robot, String clampConfigName, String bladesServoConfigName) {
        super(robot);
        clamp = robot.hardwareMap.servo.get(clampConfigName);
        bladesServo = robot.hardwareMap.crservo.get(bladesServoConfigName);


        gamepad = new CustomizableGamepad(robot);
        usesConfig = true;
    }

    @Override
    public void init() {
        resetClamp();
        //resetBlades();
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
        clampToggle.updateToggle(gamepad.getBooleanInput("Clamp"));
        if (clampToggle.getCurrentState()) {
            clampClamp();
        }
        else {
            resetClamp();
        }
    }

    @Override
    public void stop() {

    }
    public void clampClamp() {
        setClampPos(1);
    }

    public void resetClamp() {
        setClampPos(0);
    }

    public void setClampPos(double pos) {
        clamp.setPosition(Range.clip(pos,0,1));
    }

    public void powerBlades(double power) {
        bladesServo.setPower(power);
    }

    public void powerBladesTime(double power, long time) {
        powerBlades(power);
        waitTime(time);
        stopBlades();
    }

    public void stopBlades() {
        powerBlades(0);
    }
    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam("Clamp", Button.BooleanInputs.a),
        };
    }
}
