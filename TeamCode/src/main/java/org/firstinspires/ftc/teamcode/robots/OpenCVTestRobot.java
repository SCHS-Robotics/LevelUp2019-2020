package org.firstinspires.ftc.teamcode.robots;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.computervision.SconeFinderV2;

public class OpenCVTestRobot extends Robot {

    //public BLUEFoundationDetection detector;
    public SconeFinderV2 detector;
    public OpenCVTestRobot(OpMode opMode) {
        super(opMode);
        enableViewport(new Button(1, Button.BooleanInputs.noButton));
        startGui(new Button(1, Button.BooleanInputs.noButton));
        //detector = new BLUEFoundationDetection(this);
        detector = new SconeFinderV2(this);
    }
}
