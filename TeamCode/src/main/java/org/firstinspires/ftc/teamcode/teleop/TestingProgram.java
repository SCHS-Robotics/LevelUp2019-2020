package org.firstinspires.ftc.teamcode.teleop;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.TestingBot;

@StandAlone
@TeleOp(name = "ServoTester", group = "Testing")
public class TestingProgram extends BaseTeleop {
    private TestingBot robot;
    @Override
    protected Robot buildRobot() {
        robot = new TestingBot(this);
        return robot;
    }
}
