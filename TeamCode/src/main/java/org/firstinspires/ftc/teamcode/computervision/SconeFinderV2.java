package org.firstinspires.ftc.teamcode.computervision;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.VisionSubSystem;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.misc.Line;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.ximgproc.Ximgproc;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.RETR_EXTERNAL;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.THRESH_OTSU;

/**
 * A subsystem for finding skystones on the field using computer vision.
 *
 * @author Cole Savage
 * @author Isabella Valdez Palmer
 *
 * @version 2.0
 */
public class SconeFinderV2 extends VisionSubSystem {
    /**
     * The bounds for yellow in the Lab color space.
    **/
    private static final Scalar LOWER_Lab_VAL = new Scalar(0,105,170), UPPER_Lab_RANGE = new Scalar(255,170,255);
    /**
     * The size of the median blue kernel applied to the image. Must be an odd number.
     */
    private static final int MEDIAN_BLUR_KERNEL_SIZE = 3;
    /**
     * The resolution of the line detection distance accumulator in pixels.
     */
    private static final double DISTANCE_RESOLUTION_PX = 1;
    /**
     * The resolution of the angle value for detected lines.
     */
    private static final double ANGULAR_RESOLUTION = toRadians(1);
    /**
     * The minimum length of line that is detected as a line.
     */
    private static final double LINE_MIN_LENGTH = 0;
    /**
     * The size of the maximum possible gap in the block line (in pixels).
     */
    private static final double LINE_MAX_GAP = 100;
    /**
     * The minimum number of votes a potential line must have in hough space to be defined as a line.
     */
    private static final int LINE_THRESHOLD = 30;
    /**
     * The tolerance in degrees on what is considered a horizontal line.
     */
    private static final double HORIZONTAL_THRESH = 3;
    /**
     * The kernel used to perform a dilation, closing gaps in the detected skystone. It is wider than it is long in order to promote horizontal lines.
     */
    private static final Mat DILATION_KERNEL = Imgproc.getStructuringElement(MORPH_RECT, new Size(5,1));
    /**
     * How many times the corrective dilation should be performed. More iterations is often more efficient than using a larger kernel.
     */
    private static final int DILATION_ITERS = 2;
    /**
     * This is the threshold value for whether a detected region is a skystone.
     * If the difference between the min and max values of the region is greater than this, that region is a skystone.
     */
    private static final int SKYSTONE_THRESH = 200;
    /**
     * The color of the block line being drawn on the screen. This is a value in the RGBA color space.
     */
    private static final Scalar LINE_DRAW_COLOR = new Scalar(255,0,0,0);
    /**
     * The color of the skystone centers being drawn on the screen. This is a value in the RGBA color space.
     */
    private static final Scalar SKYSTONE_DRAW_COLOR = new Scalar(0,255,0,0);
    /**
     * The radius of the skystone centers being drawn on the screen in pixels.
     */
    private static final int SKYSTONE_DRAW_RADIUS = 5;
    /**
     * A list of MatOfPoint objects storing detected line segments that could be regions containing skystones.
     */
    private List<MatOfPoint> lineSegments = new ArrayList<>();
    /**
     * A list containing all regions of the black color space that could be skystones.
     */
    private List<Mat> segmentDataList = new ArrayList<>();
    /**
     * A list of all detected skystone centers.
     * This list is cached.
     */
    private List<Point> skystones = new ArrayList<>();
    /**
     * The RGB version of the input image (which is in RGBA format).
     */
    private Mat rgb = new Mat();
    /**
     * A custom one-channel color space used to detect black regions.
     */
    private Mat black = new Mat();
    /**
     * The Lab version of the input image.
     */
    private Mat lab = new Mat();
    /**
     * A binary mask defining which regions of the image are yellow.
     */
    private Mat yellowMask = new Mat();
    /**
     * The spooky, scary skeleton of the yellow binary mask.
     */
    private Mat skeleton = new Mat();
    /**
     * A Mat object containing the output of the hough probabilistic line detection algorithm
     */
    private Mat linesP = new Mat();
    /**
     * A binary mask specifying the segments of the block line that are yellow.
     */
    private Mat yellowLineComponentMask = new Mat();
    /**
     * A binary mask specifying the gaps in the yellow region of the line.
     */
    private Mat gapMask = new Mat();
    /**
     * A Mat object containing all data from the black custom color space that could be a skystone.
     */
    private Mat segmentData = new Mat();
    /**
     * A binary mask used to differentiate between skystone and non-skystone detections.
     */
    private Mat blackMask = new Mat();

    /**
     * The constructor for the SconeFinderV2 algorithm.
     *
     * @param robot The robot using this subsystem.
     */
    public SconeFinderV2(Robot robot) {
        super(robot);
    }

    @Override
    public Mat onCameraFrame(Mat input) {
        //Convert from RGBA to RGB.
        Imgproc.cvtColor(input,rgb,Imgproc.COLOR_RGBA2RGB);
        Imgproc.medianBlur(rgb.clone(),rgb,MEDIAN_BLUR_KERNEL_SIZE);

        //Start the black color space converter in a separate thread to save time.
        BlackColorspace blackConverter = new BlackColorspace(rgb,black);
        blackConverter.start();

        //Converts from the RGB color space to the Lab color space
        Imgproc.cvtColor(rgb,lab,Imgproc.COLOR_RGB2Lab);

        //Creates a binary mask specifying the locations of all pixels between the lower and upper Lab values (finds all yellow pixels).
        Core.inRange(lab, LOWER_Lab_VAL, UPPER_Lab_RANGE, yellowMask);
        lab.release();

        //Skeletonizes the yellow mask using the Guo-Hall skeletonization algorithm.
        Ximgproc.thinning(yellowMask,skeleton,Ximgproc.THINNING_GUOHALL);
        //yellowMask.;

        //Detects major lines in the skeleton using the hough probabilistic transform
        Imgproc.HoughLinesP(skeleton, linesP,
                DISTANCE_RESOLUTION_PX,
                ANGULAR_RESOLUTION,
                LINE_THRESHOLD,
                LINE_MIN_LENGTH,
                LINE_MAX_GAP);
        skeleton.release();

        //Finds the maximum and minimum y values for all detected horizontal lines.
        double minY = Double.POSITIVE_INFINITY;
        double maxY = 0;
        boolean foundLine = false;
        List<Line> lines = new ArrayList<>();
        for (int x = 0; x < linesP.rows(); x++) {
            double[] l = linesP.get(x, 0);
            Line line = new Line(new Point(l[0], l[1]), new Point(l[2], l[3]));

            //If the line is within HORIZONTAL_THRESH of 0 degrees, and is lower on the screen than the current best line, set it to the current best line.
            if(abs(line.getAngle()) < HORIZONTAL_THRESH || abs(line.getAngle()) > 180 - HORIZONTAL_THRESH) {
                lines.add(line);
                foundLine = true;

                //Update the min and max y values
                minY = min((line.getP0().y+line.getP1().y)/2.0, minY);
                maxY = max((line.getP0().y+line.getP1().y)/2.0, maxY);
            }
        }
        linesP.release();

        if(foundLine) {

            double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
            for(Line line : lines) {
                double y = (line.getP0().y+line.getP1().y)/2.0;

                x0 += line.getP0().x;
                y0 += (y/minY)*line.getP0().y;
                x1 += line.getP1().x;
                y1 += (y/minY)*line.getP1().y;
            }

            x0 /= lines.size();
            y0 /= lines.size();
            x1 /= lines.size();
            y1 /= lines.size();

            //Averages the two best found lines. Because these are usually on the top and bottom of the block line it gets the middle line.
            //Point p0 = new Point((lines.get(0).getP0().x+lines.get(1).getP0().x)/2.0,(lines.get(0).getP0().y+lines.get(1).getP0().y)/2.0);
            //Point p1 = new Point((lines.get(0).getP1().x+lines.get(1).getP1().x)/2.0,(lines.get(0).getP1().y+lines.get(1).getP1().y)/2.0);
            Line goodLine = new Line(x0,(y0+maxY)/2.0,x1,(y1+maxY)/2.0);

            //Line goodLine = (lines.get(0).getP0().y+lines.get(0).getP1().y)/2.0 > (lines.get(1).getP0().y+lines.get(1).getP1().y)/2.0 ? lines.get(0) : lines.get(1);

            //Draw the block line on the screen (goes from one end of the image to the other.
            Mat fullLineMask = Mat.zeros(rgb.size(), CvType.CV_8U);
            rgb.release();
            goodLine.drawFull(fullLineMask, new Scalar(255));

            //Performs a bitwise AND between the drawn line and the yellow binary mask to get the locations of all parts of that line that contain yellow pixels.
            Core.bitwise_and(fullLineMask, yellowMask, yellowLineComponentMask);

            //Finds all the gaps in that yellow line by finding the regions that are different between the full line and the yellow region of the line.
            Core.bitwise_xor(yellowLineComponentMask, fullLineMask, gapMask, fullLineMask);
            fullLineMask.release();
            yellowLineComponentMask.release();

            //Finds all the line segments that represent gaps in the block line.
            Imgproc.findContours(gapMask, lineSegments, new Mat(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            //Wait until the black color space converter thread is no longer running.
            waitWhile(blackConverter::isAlive);

            //For each detected gap, cut that region out of the black color space, reshape it so it only has 1 row, then add it to the list of possible skystone regions.
            for (MatOfPoint lineSegment : lineSegments) {
                Rect segmentRegion = Imgproc.boundingRect(lineSegment);
                segmentDataList.add(black.submat(segmentRegion).clone().reshape(1, 1));
                lineSegment.release();
            }
            lineSegments.clear();

            //Concatenates all detected regions together.
            Core.hconcat(segmentDataList, segmentData);

            //Clean up data list.
            for (Mat segment : segmentDataList) {
                segment.release();
            }
            segmentDataList.clear();

            //Use the otsu threshold finding algorithm to calculate the optimal threshold to split between skystone and non-skystone regions, Then apply this to the black color space to create a binary mask.
            double blackThreshold = Imgproc.threshold(segmentData, segmentData, 0, 255, THRESH_OTSU);
            Imgproc.threshold(black, blackMask, blackThreshold, 255, THRESH_BINARY);

            //Perform a bitwise AND between the gap binary mask and the black binary mask, filtering out non-skystone regions.
            Core.bitwise_and(gapMask, blackMask, gapMask);
            blackMask.release();

            //Dilate the gap mask in order to fill any holes in the detected regions.
            Imgproc.dilate(gapMask.clone(), gapMask, DILATION_KERNEL, new Point(), DILATION_ITERS);

            //Gets a list of all detected line segments.
            Imgproc.findContours(gapMask, lineSegments, new Mat(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            gapMask.release();

            //Determines which of these regions are skystones.
            skystones.clear();
            for (MatOfPoint segment : lineSegments) {
                Rect region = Imgproc.boundingRect(segment);

                //Correct for shifting effects of the dilation.
                region.x += DILATION_KERNEL.width()-1;
                region.x = Range.clip(region.x+region.width,0,black.cols()) - region.width;
                region.y += DILATION_KERNEL.height()-1;
                region.y = Range.clip(region.y+region.height,0,black.rows()) - region.height;

                //Finds the global minimum and maximum value/locations of the region.
                Core.MinMaxLocResult minMax = Core.minMaxLoc(black.submat(region));

                //If the difference between the maximum and minimum values are greater than SKYSTONE_THRESH, then the region is a skystone.
                if (minMax.maxVal - minMax.minVal > SKYSTONE_THRESH) {
                    skystones.add(new Point(region.x + region.width / 2, region.y + region.height / 2));
                }
                segment.release();
            }
            lineSegments.clear();
            black.release();

            //Draw the block line and skystones to the input image.
            goodLine.drawFull(input,LINE_DRAW_COLOR);
            for(Point skystone : skystones) {
                Imgproc.circle(input,skystone,SKYSTONE_DRAW_RADIUS,SKYSTONE_DRAW_COLOR,-1);
            }
        }

        yellowMask.release();
        return input;
    }

    @Override
    public void init() {

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

    }

    /**
     * Gets the most recent list of detected skystones.
     *
     * @return The list of detected skystones.
     */
    public List<Point> getSkystones() {
        return skystones;
    }
}
