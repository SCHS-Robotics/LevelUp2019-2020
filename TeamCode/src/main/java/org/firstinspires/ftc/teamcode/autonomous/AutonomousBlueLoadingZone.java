package org.firstinspires.ftc.teamcode.autonomous;


import android.media.MediaPlayer;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.util.annotations.MainRobot;
import com.SCHSRobotics.HAL9001.util.math.Vector;
import com.SCHSRobotics.HAL9001.util.misc.BeatBox;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.roadrunner.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.robots.Cygnus;
import org.opencv.core.Point;

import java.util.List;

import kotlin.Unit;

@Autonomous(name = "AutonomousBlueLoading", group = "competition")
public class AutonomousBlueLoadingZone extends BaseAutonomous {
    public @MainRobot Cygnus robot;
    public SampleMecanumDriveREVOptimized drive;

    private BeatBox beatBox;

    private static final double LEFT_DIVIDER = 33, RIGHT_DIVIDER = 95;
    private enum StoneState {
        LEFT(16, 2), CENTER(8, 1), RIGHT(0, 0);

        public double dX;
        public int val;
        StoneState(double dX, int val) {
            this.dX = dX;
            this.val = val;
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
            skystoneLoc = skystones.get(0).x > skystones.get(1).x ? skystones.get(0) : skystones.get(1);
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

        robot.telemetry.setAutoClear(true);

        telemetry.addData("state", state.name());
        telemetry.update();

        //Moves towards and grabs first stone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()

                        .strafeLeft(34.5)
                        .addMarker(new Vector2d(0,6),() -> {
                            robot.hugger.hugLeft();
                            robot.hugger.resetTopLeft();
                            return Unit.INSTANCE;
                        })
                        /*.addMarker(new Vector2d(-state.dX/1.5,-34.5),() -> {

                            return Unit.INSTANCE;
                        })*/
                        .build()

        );
        int delta = state == StoneState.RIGHT ? 0 : state == StoneState.CENTER ? 350 : 700;
        robot.drive.driveEncoders(new Vector(0,0.15), delta);
        robot.hugger.hugTopLeft();
        waitTime(500);
        robot.hugger.resetLeft();
        //moves first stone to building side
        waitTime(100);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(4)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(53, 25, 0))
                        .build()

        );
        // Deploys first stone
        robot.hugger.hugLeft();
        waitTime(500);
        robot.hugger.resetTopLeft();

        //Moves to second stone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .back(5)
                .setReversed(true)
                .splineTo(new Pose2d(-24 + state.dX, 32.5, 0))
                .addMarker(new Vector2d(-10, 20), () -> {
                    robot.hugger.resetTopLeft();

                    return Unit.INSTANCE;
                })
                .build()
        );
        // grabs second stone
        robot.hugger.hugTopLeft();
        waitTime(550);
        robot.hugger.resetLeft();
        waitTime(100);

        //Drop off second stone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(60, 25, 0))
                        .build()

        );
        // Prevents block pile
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(8)
                        .build()
        );
        // prepares hugger
        robot.hugger.hugLeft();
        waitTime(200);
        robot.hugger.resetTopLeft();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(8)
                        .build()
        );

        int shift1 = 8*((state.val + 2)%3);

        //moves to third stone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(shift1, 31.75, 0))
                        .addMarker(new Vector2d(-10, 20), () -> {
                            robot.hugger.resetTopLeft();
                            robot.hugger.hugLeft();
                            return Unit.INSTANCE;
                        })
                        .build()
        );
        //grabs third stone
        robot.hugger.hugTopLeft();
        waitTime(550);
        robot.hugger.resetLeft();

        //moves third stone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(60, 25, 0))
                        .build()

        );

        robot.hugger.resetTopLeft();
        robot.hugger.hugLeft();
        waitTime(600);

        //gets fourth stone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(-10, 33.5, 0))
                        .addMarker(new Vector2d(-12, 20), () -> {
                            robot.hugger.resetTopLeft();
                            robot.hugger.hugLeft();
                            return Unit.INSTANCE;
                        })
                        .build()
        );

        robot.hugger.hugTopLeft();
        waitTime(500);
        robot.hugger.resetLeft();
        //moves fourth block
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(60, 25, 0))
                        .build()

        );
        robot.hugger.resetTopLeft();
        robot.hugger.hugLeft();
        robot.hugger.resetTopLeft();
        //prevents block pileup
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(6)
                        .strafeRight(6)
                        .build()
        );
        robot.hugger.resetLeft();

        //parks
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(36, 28, 0))
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

        robot.hugger.hugLeft();
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

        robot.hugger.resetLeft();
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

        robot.hugger.hugLeft();
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
        robot.hugger.resetLeft();

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
        robot.hugger.hugLeft();

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
        robot.hugger.resetLeft();

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
        robot.skystoneDetector.startVision();
    }

    @Override
    protected void onStop() {
        beatBox.stopSong("Spooky");
    }
}
