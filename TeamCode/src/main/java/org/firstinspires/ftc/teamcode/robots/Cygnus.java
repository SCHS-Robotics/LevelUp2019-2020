package org.firstinspires.ftc.teamcode.robots;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.computervision.SconeFinderV2;
import org.firstinspires.ftc.teamcode.subsystems.CapstonePlopper;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Hugger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LineDetector;

/**
 * The robot for the Level Up 2019-2020 season, called Cygnus.
 *
 * @author Cole Savage
 * @author Jack Kinney
 * @author Isabella Valdez Palmer
 * @author Ella Player
 * @author Tristan Tran
 * @author Jared Smith
 */
public class Cygnus extends Robot {

    //The MecanumDrive subsystem.
    public MechanumDrive drive;
    //The Intake subsystem.
    public Intake intake;
    //The Line Detector subsystem.
    public LineDetector lineDetector;
    //The capstone release subsystem.
    public CapstonePlopper plopper;
    //The foundation mover subsystem
    public FoundationMover mover;
    //The skystone attractor subsystem, nicknamed the block hugger.
    public Hugger hugger;
    //The skystone detector subsystem.
    public SconeFinderV2 skystoneDetector;

    /**
     * The constructor for the Cygnus Robot.
     *
     * @param opMode The opmode the robot will run.
     */
    public Cygnus(OpMode opMode) {
        super(opMode);

        //Turns on the OpenCV viewport. Provided button will be used to cycle between pipelines.
        enableViewport(new Button(1, Button.BooleanInputs.noButton));

        //Creates all subsystems.
        drive = new MechanumDrive(this,"topLeft","topRight","botLeft","botRight");
        intake = new Intake(this, "intakeLeft", "intakeRight", "pull");
        lineDetector = new LineDetector(this, "colorSensor");
        plopper = new CapstonePlopper(this,"plop");
        mover = new FoundationMover(this,"leftLatch", "rightLatch");
        hugger = new Hugger(this, "huggerLeft", "huggerRight","topLeftHugger","topRightHugger");
        skystoneDetector = new SconeFinderV2(this);
    }
}
