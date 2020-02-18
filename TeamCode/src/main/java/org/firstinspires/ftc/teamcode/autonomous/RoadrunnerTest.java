package org.firstinspires.ftc.teamcode.autonomous;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.util.annotations.MainRobot;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.robots.Cygnus;

@Autonomous(name = "Wham Bam Kabam Shazam", group = "competition")
public class RoadrunnerTest extends BaseAutonomous {
    public @MainRobot Cygnus robot;

    public SampleMecanumDriveREVOptimized drive;
    @Override
    public void main() {
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(15, 15, 0))
                        .splineTo(new Pose2d(30, -15, 0))
                        .build()

        );
    }

    @Override
    protected void onInit() {
        drive = new SampleMecanumDriveREVOptimized(robot.hardwareMap);
    }
}
