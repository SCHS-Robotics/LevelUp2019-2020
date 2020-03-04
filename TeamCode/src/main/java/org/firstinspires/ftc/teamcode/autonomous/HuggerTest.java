package org.firstinspires.ftc.teamcode.autonomous;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.util.annotations.MainRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.robots.Cygnus;

@Autonomous(name = "HuggerTest", group = "competition")
public class HuggerTest extends BaseAutonomous {
    public @MainRobot Cygnus robot;

    public SampleMecanumDriveREVOptimized drive;
    @Override
    public void main() {
        robot.hugger.hugRight();
        waitTime(3000);
        robot.hugger.hugTopRight();
        waitTime(750);
        robot.hugger.resetRight();
        waitTime(750);
        /*
        for (int i = 0; i < 5; i++) {
            robot.hugger.hugLeft();
            waitTime(750);
            robot.hugger.resetLeft();
            waitTime(750);
            robot.hugger.hugTopLeft();
            waitTime(750);
            robot.hugger.resetTopLeft();
            waitTime(750);
        }
        */

    }
}
