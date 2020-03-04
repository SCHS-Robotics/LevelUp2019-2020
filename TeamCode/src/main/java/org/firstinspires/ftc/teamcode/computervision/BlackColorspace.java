package org.firstinspires.ftc.teamcode.computervision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * A thread for generating a 1-channel black color space from an RGB image.
 *
 * @author Cole Savage
 */
public class BlackColorspace extends Thread {
    /**
     * The input image in RGB format.
     */
    private Mat rgb;
    /**
     * The output image.
     */
    private Mat output;
    /**
     * The grayscale version of the RGB image.
     */
    private Mat gray = new Mat();
    /**
     * The difference between the RGB R and G channels.
     */
    private Mat RG = new Mat();
    /**
     * The difference between the RGB R and B channels.
     */
    private Mat RB = new Mat();
    /**
     * The difference between the RGB G and B channels.
     */
    private Mat GB = new Mat();
    /**
     * A Mat object containing the custom black color space. This color space is based on each RGB pixel's euclidean distance from black (0,0,0).
     */
    private Mat rgbDistance = new Mat();
    /**
     * A list containing 3 Mat objects, each a channel of an RGB image.
     */
    private List<Mat> rgbLst = new ArrayList<>();

    /**
     * The constructor for the BlackColorspace converter.
     *
     * @param rgb The input RGB image.
     * @param output The output 1-channel custom black color space.
     */
    public BlackColorspace(Mat rgb, Mat output) {
        this.rgb = rgb.clone();
        this.output = output;
    }

    @Override
    public void run() {
        //Converts the RGB input image to grayscale.
        Imgproc.cvtColor(rgb,gray,Imgproc.COLOR_RGB2GRAY);
        gray.convertTo(gray,CvType.CV_32F);

        //Converts the rgbDistance Mat object's values to 32 bit unsigned floating point numbers.
        rgbDistance.convertTo(rgbDistance,CvType.CV_32F);

        //Spits the RGB image into a list of 3 Mat objects each containing a channel from the RGB image.
        Core.split(rgb,rgbLst);
        rgb.release();

        //Converts the channel values from 8 bit unsigned integers to 32 bit signed floating point numbers.
        rgbLst.get(0).convertTo(rgbLst.get(0), CvType.CV_32S);
        rgbLst.get(1).convertTo(rgbLst.get(1),CvType.CV_32S);
        rgbLst.get(2).convertTo(rgbLst.get(2),CvType.CV_32S);

        //Calculate the RG, RB, and GB Mats.
        Core.subtract(rgbLst.get(0),rgbLst.get(1),RG);
        Core.subtract(rgbLst.get(0),rgbLst.get(2),RB);
        Core.subtract(rgbLst.get(1),rgbLst.get(2),GB);

        //Cleans up channels in rgbLst.
        rgbLst.get(0).release();
        rgbLst.get(1).release();
        rgbLst.get(2).release();
        rgbLst.clear();

        //Square the RG, RB, and GB Mat objects.
        Core.pow(RG,2,RG);
        Core.pow(RB,2,RB);
        Core.pow(GB,2,GB);

        //Converts the elements in the RG, RB, and GB Mat objects to 32 bit unsigned floating point numbers.
        RG.convertTo(RG,CvType.CV_32F);
        RB.convertTo(RB,CvType.CV_32F);
        GB.convertTo(GB,CvType.CV_32F);

        //Uses the distance formula on the RG, RB, and GB Mat objects.
        Core.add(RG,RB,rgbDistance,new Mat(),CvType.CV_32F);
        RG.release();
        RB.release();
        Core.add(rgbDistance,GB,rgbDistance,new Mat(),CvType.CV_32F);
        GB.release();
        Core.sqrt(rgbDistance.clone(),rgbDistance);

        //Scales down the rgbDistance and gray images so that when added together their maximum values equal 255, the max value that can be stored in 8 bits.
        Core.multiply(rgbDistance,new Scalar(127.5/255.0),rgbDistance);
        Core.multiply(gray,new Scalar(127.5/255.0),gray);

        //Add the gray image and the rgbDistance image.
        Core.add(rgbDistance,gray,rgbDistance);
        gray.release();

        //Perform a bitwise not of rgbDistance.
        rgbDistance.convertTo(rgbDistance,CvType.CV_8U);
        Core.bitwise_not(rgbDistance,rgbDistance);
        rgbDistance.convertTo(rgbDistance,CvType.CV_32F);

        //Rescale the rgbDistance image between 0 and 1, then raise it to the third power and scale it back up to reduce noise.
        Core.multiply(rgbDistance,new Scalar(1.0/255),rgbDistance);
        Core.pow(rgbDistance,3,rgbDistance);
        Core.multiply(rgbDistance,new Scalar(255),rgbDistance);

        //Convert rgbDistance's elements back to 8 bit integers and copy it to the output Mat.
        rgbDistance.convertTo(rgbDistance,CvType.CV_8U);
        rgbDistance.copyTo(output);
        rgbDistance.release();
    }
}