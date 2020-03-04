package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * The line detector subsystem.
 * Used for detecting the line under the skybridge.
 *
 * @author Cole Savage
 * @author Jack Kinney
 */
public class LineDetector extends SubSystem {

    //The colorsensor used to detect the line. Currently the colorsensor is a Rev V2 colorsensor.
    private ColorSensor colorSensor;
    //The center of the blue color in the HSV colorspace.
    private static final double BLUE_CENTER = 210;
    //The radius of the color detection for red and blue.
    private static double BLUE_RADIUS = 15.0;
    private static double RED_RADIUS = 15.0;

    /**
     * The constructor for the line detector subsystem.
     *
     * @param robot The robot using the subsystem.
     * @param colorSensorConfig The configuration name for the color sensor.
     */
    public LineDetector(Robot robot, String colorSensorConfig) {
        super(robot);
        colorSensor = robot.hardwareMap.colorSensor.get(colorSensorConfig);
    }

    @Override
    public void init() {
        //Turns on the colorsensor LED for increased consistency.
        enableLed();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        //Turns off the LED.
        disableLed();
    }

    /**
     * Turns on the LED.
     */
    public void enableLed() {
        colorSensor.enableLed(true);
    }

    /**
     * Turns off the LED.
     */
    public void disableLed() {
        colorSensor.enableLed(false);
    }

    /**
     * Gets the current value of the H channel in the HSV format.
     *
     * @return The color of the current surface under the color sensor.
     */
    public double getColor() {
        double[] hsv = getHSV();
        return hsv[0];
    }

    /**
     * Gets the current HSV value of the surface under the color sensor.
     *
     * @return The current HSV value of the surface under the color sensor.
     */
    public double[] getHSV() {
        float[] hsvFloat = new float[3];
        double[] hsv = new double[3];
        Color.RGBToHSV((colorSensor.red() * 255), (colorSensor.green() * 255), (colorSensor.blue() * 255), hsvFloat);
        for (int i = 0; i < hsv.length; i++) {
            hsv[i] = (double) hsvFloat[i];
        }
        return hsv;
    }

    /**
     * Gets the RGB value of the surface currently under the color sensor.
     *
     * @return The RGB value of the surface currently under the color sensor.
     */
    public double[] getRGB() {
        return new double[] {colorSensor.red(),colorSensor.green(),colorSensor.blue()};
    }

    /**
     * Gets whether red has been detected by the color sensor.
     *
     * @return Whether red has been detected by the color sensor.
     */
    public boolean isRedDetected() {
        double color = getColor();
        return color < RED_RADIUS || color > 360 - RED_RADIUS;
    }

    /**
     * Gets whether blue has been detected by the color sensor.
     *
     * @return Whether blue has been detected by the color sensor.
     */
    public boolean isBlueDetected() {
        double color = getColor();
        return Math.abs(color - BLUE_CENTER) < BLUE_RADIUS;
    }

    /**
     * Sets the radius of the red detector.
     *
     * @param redRadius The radius of the red detector.
     */
    public void setRedRadius(double redRadius) {
        RED_RADIUS = redRadius;
    }

    /**
     * Sets the radius of the blue detector.
     *
     * @param blueRadius The radius of the blue detector.
     */
    public void setBlueRadius(double blueRadius) {
        BLUE_RADIUS = blueRadius;
    }
}