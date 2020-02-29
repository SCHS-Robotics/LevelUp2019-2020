package org.firstinspires.ftc.teamcode.subsystems;


import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.AutonomousConfig;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigData;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *
 */
public class Intake extends SubSystem {

    //The names of the control buttons for the intake system.
    private static final String INTAKE = "intake", OUTTAKE = "outtake";
    //The DcMotors connected to the compliant wheels that directly intake the block.
    private DcMotor leftIntakeMotor, rightIntakeMotor;
    //The DcMotor used to transfer the block into the upper tray.
    private DcMotor transferMotor;
    //A multiplier that is multiplied by the power in order to increase or decrease the speed of the transfer motor.
    private double transferPowerMultiplier;
    //The gamepad used to control the intake.
    private CustomizableGamepad gamepad;
    //The power at which the intake wheels will spin.
    private double intakePower;

    /**
     * A constructor for the intake subsystem.
     *
     * @param robot The robot using this subsystem.
     * @param leftIntakeConfig The config name of the left intake motor.
     * @param rightIntakeConfig The config name of the right intake motor.
     * @param transferConfig The config name of the transfer motor.
     * @param intakeButton The button used to intake blocks.
     * @param outtakeButton The button used to outtake blocks.
     * @param intakePower The power at which the intake will spin.
     * @param transferPowerMultiplier The multiplier applied to the transfer wheel power.
     */
    public Intake(Robot robot, String leftIntakeConfig, String rightIntakeConfig, String transferConfig, Button intakeButton, Button outtakeButton, double intakePower, double transferPowerMultiplier) {
        super(robot);

        //Create a new customizable gamepad.
        gamepad = new CustomizableGamepad(robot);
        //Prevent against negative powers.
        this.intakePower = Math.abs(intakePower);

        //Hardwaremaps all the DcMotors.
        leftIntakeMotor = robot.hardwareMap.dcMotor.get(leftIntakeConfig);
        rightIntakeMotor = robot.hardwareMap.dcMotor.get(rightIntakeConfig);
        transferMotor = robot.hardwareMap.dcMotor.get(transferConfig);

        //Reverses the direction of the transfer motor in order to make the powers positive for transferring blocks upward.
        transferMotor.setDirection(DcMotor.Direction.REVERSE);

        //Sets what the motors should do when given zero power. The intake motors will actively brake, whole the transfer motor will float to a stop.
        leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Adds the buttons to the gamepad.
        gamepad.addButton(INTAKE, intakeButton);
        gamepad.addButton(OUTTAKE, outtakeButton);

        //Sets the transferPowerMultiplier.
        this.transferPowerMultiplier = transferPowerMultiplier;
    }

    /**
     * A constructor for the Intake subsystem. This constructor uses the HAL 9001 config system.
     *
     * @param robot The robot using this subsystem.
     * @param leftIntakeConfig The config name of the left intake motor.
     * @param rightIntakeConfig The config name of the right intake motor.
     * @param transferConfig The config name of the transfer motor.
     */
    public Intake(Robot robot, String leftIntakeConfig, String rightIntakeConfig, String transferConfig) {
        super(robot);

        //Create a new customizable gamepad.
        gamepad = new CustomizableGamepad(robot);

        //Hardwaremaps all the DcMotors.
        leftIntakeMotor = robot.hardwareMap.dcMotor.get(leftIntakeConfig);
        rightIntakeMotor = robot.hardwareMap.dcMotor.get(rightIntakeConfig);
        transferMotor = robot.hardwareMap.dcMotor.get(transferConfig);

        //Reverses the direction of the transfer motor in order to make the powers positive for transferring blocks upward.
        transferMotor.setDirection(DcMotor.Direction.REVERSE);

        //Sets what the motors should do when given zero power. The intake motors will actively brake, whole the transfer motor will float to a stop.
        leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Tells the subsystem to use the HAL9001 config system. This instance variable is present in the subsystem superclass.
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
        //If the subsyste uses config, pull all the config settings from the config.
        if (usesConfig) {
            //Get the configured controller.
            gamepad = robot.pullControls(this);

            //Get the non-button data from the config system and set the appropriate parameters.
            ConfigData data = robot.pullNonGamepad(this);
            intakePower = data.getData("IntakePower", Double.class);
            transferPowerMultiplier = data.getData("TransferPowerMultiplier", Double.class);
        }
    }

    @Override
    public void handle() {
        //Determine whether to intake or outtake a block.
        boolean intakeBool = gamepad.getBooleanInput(INTAKE);
        boolean outtakeBool = gamepad.getBooleanInput(OUTTAKE);

        //Intakes or outakes a block.
        intake(intakeBool ? intakePower : outtakeBool ? -intakePower : 0);
    }

    @Override
    public void stop() {
        //Stops the intake if it is running.
        stopIntake();
    }

    /**
     * Runs the intake at the specified speed.
     *
     * @param power The power to run the intake at.
     */
    public void intake(double power) {
        leftIntakeMotor.setPower(power);
        rightIntakeMotor.setPower(power);
        transferMotor.setPower(transferPowerMultiplier*power);

    }

    /**
     * Runs the intake at the specified speed for the specified time.
     *
     * @param power The power to run the intake at.
     * @param time The amount of time to run the intake for in milliseconds.
     */
    public void intakeTime(double power, long time) {
        intake(power);
        waitTime(time);
        stopIntake();
    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake() {
        intake(0);
    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam(INTAKE, Button.BooleanInputs.bool_left_trigger),
                new ConfigParam(OUTTAKE, Button.BooleanInputs.bool_right_trigger),
                new ConfigParam("IntakePower",ConfigParam.numberMap(0,1,0.1),1.0),
                new ConfigParam("TransferPowerMultiplier", ConfigParam.numberMap(0,1,0.1),1.0)
        };
    }

    @AutonomousConfig
    public static ConfigParam[] autonomousConfig() {
        return new ConfigParam[] {
                new ConfigParam("IntakePower",ConfigParam.numberMap(0,1,0.1),1.0),
                new ConfigParam("TransferPowerMultiplier", ConfigParam.numberMap(0,1,0.1),1.0)
        };
    }
}
