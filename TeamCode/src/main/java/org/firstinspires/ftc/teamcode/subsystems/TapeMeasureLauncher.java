package org.firstinspires.ftc.teamcode.subsystems;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.CRServo;

import org.jetbrains.annotations.NotNull;

public class TapeMeasureLauncher extends SubSystem {
    private CRServo crServo;
    private double power;

    private CustomizableGamepad gamepad;
    public  TapeMeasureLauncher(@NotNull Robot robot, String LaunchServoConfig) {
        super(robot);
        crServo = robot.hardwareMap.crservo.get(LaunchServoConfig);

        gamepad = new CustomizableGamepad(robot);
        usesConfig = true;
    }

    @Override
    public void init() {

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
        boolean launchBool = gamepad.getBooleanInput("Launch");
        boolean retractBool = gamepad.getBooleanInput("Retract");

        double power = (launchBool || retractBool) ? ((launchBool) ? 0.5 : -0.5) : 0;

        crServo.setPower(power);
    }

    public void launch(double power) {
        crServo.setPower(power);
    }

    public void retract(double power) {
        crServo.setPower(-power);
    }

    @Override
    public void stop() {

    }


    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam("Launch", Button.BooleanInputs.dpad_right),
                new ConfigParam("Retract", Button.BooleanInputs.dpad_left),

        };
    }

}
