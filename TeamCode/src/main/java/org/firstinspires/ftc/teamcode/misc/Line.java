package org.firstinspires.ftc.teamcode.misc;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;
import static org.opencv.imgproc.Imgproc.LINE_AA;

public class Line {
    private Point p0, p1;

    public Line(double x0, double y0, double x1, double y1) {
        p0 = new Point(x0,y0);
        p1 = new Point(x1,y1);
    }

    public Line(Point p0, Point p1) {
        this.p0 = p0;
        this.p1 = p1;
    }

    public double getLength() {
        return hypot(p1.x-p0.x,p1.y-p0.y);
    }

    public void draw(Mat drawMat, Scalar color, int thickness, int lineStyle, int shift) {
        Imgproc.line(drawMat, p0, p1, color, thickness, lineStyle, shift);
    }

    public void draw(Mat drawMat) {
        draw(drawMat,new Scalar(255,0,0),3, LINE_AA,0);
    }

    public void draw(Mat drawMat, Scalar color) {
        draw(drawMat,color,3,LINE_AA,0);
    }

    public void drawFull(Mat drawMat, Scalar color) {
        double vectorX = abs(p1.x-p0.x);
        double vectorY = abs(p1.y-p0.y);

        Imgproc.line(drawMat, new Point(p0.x-200*vectorX,p0.y-200*vectorY), new Point(p1.x+200*vectorX, p1.y+200*vectorY), color, 5, LINE_AA, 0);
    }

    public double getAngle(boolean inDegrees) {
        return inDegrees ? toDegrees(atan2(p1.y-p0.y,p1.x-p0.x)) : atan2(p1.y-p0.y,p1.x-p0.x);
    }

    public double getAngle() {
        return getAngle(true);
    }

    public Point getP0() {
        return p0;
    }

    public Point getP1() {
        return p1;
    }
}
