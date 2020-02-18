package org.firstinspires.ftc.teamcode.teleop;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.util.annotations.MainRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.OpenCVTestRobot;

@TeleOp(name = "OpenCV Testing", group = "testing")
public class OpenCVTesting extends BaseTeleop {
    public @MainRobot
    OpenCVTestRobot robot;

    @Override
    public void onStart() {
        robot.detector.startVision();
    }
}