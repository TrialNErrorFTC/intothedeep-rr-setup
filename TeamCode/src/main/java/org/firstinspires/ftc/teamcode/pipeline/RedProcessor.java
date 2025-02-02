package org.firstinspires.ftc.teamcode.pipeline;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class RedProcessor implements VisionProcessor {
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Not useful in this case, but we do need to implement it either way
    }

    public class Alignment {
        public double deltaX;
        public double deltaY;
        public double angle;
        public Alignment(double x, double y, double a) {
            this.deltaX = x;
            this.deltaY = y;
            this.angle = a;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame == null) return null;

        double BLOCK_HEIGHT = 100;
        double BLOCK_WIDTH = 50;
        String COLOR = "RED";
        double INCHES_PER_PIXEL = 0.014625;

        // Set a point at 1/3 from the bottom of the image and 1/2 from the left.
        Point target_point = new Point(frame.cols() / 2.0, frame.rows() * 2 / 3.0);


        // Convert the frame to HSV
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        Mat output = new Mat();

        int erode_int = 5;
        int dilate_int = 9;

        if (COLOR.equals("RED")) {
            Scalar lower1 = new Scalar(0, 0, 30);
            Scalar upper1 = new Scalar(10, 255, 255);
            Scalar lower2 = new Scalar(170, 20, 100);
            Scalar upper2 = new Scalar(179, 255, 255);

            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(frame, lower1, upper1, mask1);
            Core.inRange(frame, lower2, upper2, mask2);

            Mat mask = new Mat();
            Core.add(mask1, mask2, mask);

            frame.copyTo(output, mask);
        } else if (COLOR.equals("BLUE")) {
            Scalar lower = new Scalar(100, 50, 0);
            Scalar upper = new Scalar(140, 255, 255);
            erode_int = 9;
            dilate_int = 13;
            Mat mask = new Mat();
            Core.inRange(frame, lower, upper, mask);

            frame.copyTo(output, mask);
        } else if (COLOR.equals("YELLOW")) {
            Scalar lower = new Scalar(10, 90, 50);
            Scalar upper = new Scalar(50, 255, 255);

            Mat mask = new Mat();
            Core.inRange(frame, lower, upper, mask);

            frame.copyTo(output, mask);
        }

        // Ignore the bottom 1/3 of the image.
//        Rect roi = new Rect(0, 0, frame.cols(), frame.rows() * 2 / 3);
//        output = new Mat(output, roi);

        Imgproc.erode(output, output, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(erode_int, erode_int)));
        Imgproc.dilate(output, output, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(dilate_int, dilate_int)));

        // Convert the output to grayscale
        Imgproc.cvtColor(output, output, Imgproc.COLOR_BGR2GRAY);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        List<RotatedRect> rects = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(output, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Point imgCenter = new Point(frame.cols() / 2.0, frame.rows() / 2.0);
        double minDist = Double.MAX_VALUE;
        RotatedRect minRect = null;

        // Find the closest rectangle to the center of the image.
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, 0.02 * Imgproc.arcLength(contour2f, true), true);

            // We only care about rectangles with 4 corners that are close to the expected size.
            if (approx.toList().size() >= 4) {
                RotatedRect rect = Imgproc.minAreaRect(approx);
                // Check if the rectangle is bigger than the block size.
                if (rect.size.height < BLOCK_HEIGHT || rect.size.width < BLOCK_WIDTH) {
                    continue;
                }
                Point[] points = new Point[4];
                rect.points(points);

                double dist = 0;
                for (Point p : points) {
                    dist += Math.sqrt(Math.pow(p.x - target_point.x, 2) + Math.pow(p.y - target_point.y, 2));
                }

                if (dist < minDist) {
                    minDist = dist;
                    minRect = rect;
                }

                // rects.add(rect);
            }
        }

        // Test color detection.
        // output.copyTo(frame);

        // Covert back to RGB.
        //Imgproc.cvtColor(frame, frame, Imgproc.COLOR_HSV2RGB);

        // Draw a rectangle around all the found rectangles.
//        for (RotatedRect rect : rects) {
//            Point[] points = new Point[4];
//            rect.points(points);
//
//            for (int i = 0; i < 4; i++) {
//                Imgproc.line(frame, points[i], points[(i + 1) % 4], new Scalar(0, 255, 0), 2);
//            }
//
//            // Print the center and angle of the rectangle with 2 decimal places.
//            Point center = rect.center;
//            // Normalize the angle to be the angle from a vertical line.
//            // Draw a vertical line from the center of the rectangle.
//            Imgproc.line(frame, center, new Point(center.x, center.y - 50), new Scalar(0, 0, 255), 2);
//            double angle = rect.angle;
//            // If the width is greater than the height, the angle is off by 90 degrees.
//            if (rect.size.width > rect.size.height) {
//                angle += 90;
//            }
//            // Make sure the angle is between -180 and 180 normalized to be from a vertical line.
//            if (angle > 90) {
//                angle -= 180;
//            } else if (angle < -90) {
//                angle += 180;
//            }
//            Imgproc.line(frame, center, new Point(center.x + 50 * Math.cos(Math.toRadians(angle)), center.y + 50 * Math.sin(Math.toRadians(angle))), new Scalar(255, 0, 0), 2);
//
//            // Print the center and angle of the rectangle with 2 decimal places.
//            String text = String.format("Center: (%.2f, %.2f), Angle: %.2f", center.x, center.y, angle);
//            Imgproc.putText(frame, text, new Point(center.x, center.y + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//            // Print the width and height of the rectangle.
//            text = String.format("Width: %.2f, Height: %.2f", rect.size.width, rect.size.height);
//            Imgproc.putText(frame, text, new Point(center.x, center.y + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//
//            // Draw a line from the target point to the center of the closest rectangle.
//            if (minRect != null) {
//                Imgproc.line(frame, target_point, minRect.center, new Scalar(255, 100, 255), 2);
//            }
//            // Print the delta x and y from the target point to the center of the closest rectangle.
//            if (minRect != null) {
//                text = String.format("Delta X: %.2f, Delta Y: %.2f", target_point.x - minRect.center.x, target_point.y - minRect.center.y);
//                Imgproc.putText(frame, text, new Point(center.x, center.y + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//            }
//
//        }
        double delta_x = 0;
        double delta_y = 0;
        double angle = 0;

        if (minRect != null) {
            delta_x = target_point.x - minRect.center.x;
            delta_y = target_point.y - minRect.center.y;
            angle = minRect.angle;
            // If the width is greater than the height, the angle is off by 90 degrees.
            if (minRect.size.width > minRect.size.height) {
                angle += 90;
            }
            // Make sure the angle is between -180 and 180 normalized to be from a vertical line.
            if (angle > 90) {
                angle -= 180;
            } else if (angle < -90) {
                angle += 180;
            }

            // Convert pixels to inches to move.
            delta_x *= INCHES_PER_PIXEL;
            delta_y *= INCHES_PER_PIXEL;
        }

        return new Alignment(delta_x, delta_y, angle);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
