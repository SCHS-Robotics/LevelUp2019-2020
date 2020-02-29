package org.firstinspires.ftc.teamcode.autonomous;


import android.media.MediaPlayer;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.util.annotations.MainRobot;
import com.SCHSRobotics.HAL9001.util.misc.BeatBox;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.roadrunner.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.robots.Cygnus;

import kotlin.Unit;

@Autonomous(name = "AutonomousBlueLoading", group = "competition")
public class AutonomousBlueLoadingZone extends BaseAutonomous {
    public @MainRobot Cygnus robot;
    public SampleMecanumDriveREVOptimized drive;

    private BeatBox beatBox;

    private static final double LEFT_DIVIDER = 118, RIGHT_DIVIDER = 182;
    private enum StoneState {
        LEFT(5.5), CENTER(-2.5), RIGHT(-10.5);

        public double dX;
        StoneState(double dX) {
            this.dX = dX;
        }
    }
    private StoneState state;

    @Override
    public void main() {
        /*moves down left  below stones
         * moves up and intakes / stores a stone
         * moves back out of stones
         * strafes right then drive into subcase 1 or 2
        robot.drive.driveEncoders(new Vector(0.3,Math.toRadians(0), Vector.CoordinateType.POLAR), 1550);
        waitTime(500);
        robot.hugger.hug();
        waitTime(500);
        robot.drive.driveEncoders(new Vector(-0.5,Math.toRadians(-10), Vector.CoordinateType.POLAR), 2200);
        waitTime(500);
        robot.drive.driveEncoders(new Vector(0.3,Math.toRadia+ns(0), Vector.CoordinateType.POLAR), 100);
        waitTime(500);
        robot.drive.driveEncoders(new Vector(-0.3,Math.toRadians(-90), Vector.CoordinateType.POLAR), 2600);
        waitTime(500);
        robot.hugger.reset();
        waitTime(500);
        robot.drive.drive(new Vector(-0.1,Math.toRadians(90), Vector.CoordinateType.POLAR));
        waitWhile(() -> !robot.lineDetector.isBlueDetected());
        robot.drive.stopAllMotors();*/
/*

        waitUntil(() -> robot.skystoneDetector.getSkystones().size() > 0);
        robot.skystoneDetector.stopVision();

        robot.telemetry.addLine("Skystone Detected");
        robot.telemetry.update();

        List<Point> skystones = robot.skystoneDetector.getSkystones();
        Point skystoneLoc;
        if(skystones.size() == 1) {
            skystoneLoc = skystones.get(0);
        }
        else {
            skystoneLoc = skystones.get(0).x < skystones.get(1).x ? skystones.get(0) : skystones.get(1);
        }

        if(skystoneLoc.x < LEFT_DIVIDER) {
            state = StoneState.LEFT;
        }
        else if(skystoneLoc.x >= LEFT_DIVIDER && skystoneLoc.x <= RIGHT_DIVIDER) {
            state = StoneState.CENTER;
        }
        else {
            state = StoneState.RIGHT;
        }

        state = StoneState.CENTER;
        robot.telemetry.setAutoClear(true);
/*
        telemetry.addData("state", state.name());
        telemetry.update();
        if (state.dX < 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(-state.dX)//splineTo(new Pose2d(32.25, 24   , toRadians(90))))
                            .build()

            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(state.dX)//splineTo(new Pose2d(32.25, 24   , toRadians(90))))
                            .build()


            );
        }
  */
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(35.5)
                        .addMarker(new Vector2d(0,6),() -> {
                            robot.hugger.hugLeft();
                            robot.hugger.resetTopLeft();
                            return Unit.INSTANCE;
                        })
                        .addMarker(new Vector2d(0, 29), () -> {
                            robot.hugger.hugTopLeft();
                            //waitTime(2000);

                            return Unit.INSTANCE;
                        })


                        .build()

        );
        waitTime(500);
        robot.hugger.resetLeft();
        waitTime(400);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(-30, 15, 0))
                        .addMarker(new Vector2d(-25, 15), () -> {
                            robot.hugger.resetLeft();
                            return Unit.INSTANCE;
                        })

                        .build()
        );

        waitTime(250);
        robot.hugger.resetTopLeft();
        waitTime(250);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .forward(5)
                .splineTo(new Pose2d(0, 30, 0))
                .build()
        );
        /*
        waitUntil(() -> robot.skystoneDetector.getSkystones().size() > 0);
        robot.skystoneDetector.stopVision();

        robot.telemetry.addLine("Skystone Detected");
        robot.telemetry.update();

        List<Point> skystones = robot.skystoneDetector.getSkystones();
        Point skystoneLoc;
        if(skystones.size() == 1) {
            skystoneLoc = skystones.get(0);
        }
        else {
            skystoneLoc = skystones.get(0).x < skystones.get(1).x ? skystones.get(0) : skystones.get(1);
        }

        if(skystoneLoc.x < LEFT_DIVIDER) {
            state = StoneState.LEFT;
        }
        else if(skystoneLoc.x >= LEFT_DIVIDER && skystoneLoc.x <= RIGHT_DIVIDER) {
            state = StoneState.CENTER;
        }
        else {
            state = StoneState.RIGHT;
        }

        telemetry.addData("state", state.name());
        telemetry.update();
        if (state.dX < 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(-state.dX)//splineTo(new Pose2d(32.25, 24   , toRadians(90))))
                            .build()

            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(state.dX)//splineTo(new Pose2d(32.25, 24   , toRadians(90))))
                            .build()


            );
        }

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(37)//splineTo(new Pose2d(32.25, 24   , toRadians(90))))
                        .build()

        );

        robot.hugger.hugRight();
        waitTime(400);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(14)
                        .build()
        );
        waitTime(200);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(50)
                        .build()
        );

        robot.hugger.resetRight();
        waitTime(200);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(74)
                        .build()

        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(15.5)
                        .build()

        );

        robot.hugger.hugRight();
        waitTime(500);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(17.5)
                        .build()
        );

        waitTime(200);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .forward(64-state.dX)
                .build()
        );
        robot.hugger.resetRight();

        waitTime(200);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(47.5-state.dX)
                        .build()

        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(22)
                        .build()

        );
        robot.hugger.hugRight();

        waitTime(200);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(28)
                        .build()
        );
        //drive.turn(toRadians(15));
        waitTime(200);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(58)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(8)
                        .build()
        );
        robot.hugger.resetRight();

        waitTime(200);
        robot.drive.drive(new Vector(0,-0.5));
        waitUntil(() -> robot.lineDetector.isRedDetected() || robot.lineDetector.isBlueDetected());
        robot.drive.stopAllMotors();
        //robot.intake.intake(1);
        //robot.drive.driveDistance(new Vector(0,0.5), 25, Units.CENTIMETERS);
        //robot.intake.intake(0);
        //robot.drive.driveDistance(new Vector(-0.5,0), 25, Units.CENTIMETERS);
        //moves into subcase 1 or 2
         */
    }

    @Override
    protected void onInit() {
        drive = new SampleMecanumDriveREVOptimized(robot.hardwareMap);
        beatBox = new BeatBox();
        beatBox.addSong("Spooky", MediaPlayer.create(robot.hardwareMap.appContext,R.raw.ggthemebest));
        beatBox.baseBoost("Spooky",100);

        beatBox.playSong("Spooky");
        //robot.hugger.reset();
        //robot.skystoneDetector.startVision();
    }

    @Override
    protected void onStop() {
        beatBox.stopSong("Spooky");
    }
}
