package org.firstinspires.ftc.teamcode.autonomous;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.util.annotations.LinkTo;
import com.SCHSRobotics.HAL9001.util.annotations.MainRobot;
import com.SCHSRobotics.HAL9001.util.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.Cygnus;

@Autonomous(name = "Autonomous", group = "competition")
@LinkTo(destination = "Teleop")
public class AutonomousProgram extends BaseAutonomous {
    public @MainRobot Cygnus robot;

    @Override
    public void main() {
        robot.drive.drive(new Vector(0,0.2));
        waitUntil(() -> robot.lineDetector.isRedDetected() || robot.lineDetector.isBlueDetected());
        robot.drive.stopAllMotors();
    }
}
