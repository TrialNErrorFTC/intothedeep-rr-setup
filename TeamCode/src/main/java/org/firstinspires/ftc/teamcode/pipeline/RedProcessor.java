package org.firstinspires.ftc.teamcode.pipeline;

import static org.opencv.imgproc.Imgproc.getRotationMatrix2D;

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
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import android.graphics.Paint;
import android.graphics.Color;


public class RedProcessor implements VisionProcessor {
    // Create a new Paint object
    Paint paint = new Paint();
    List<MatOfPoint> contours = new ArrayList<>();


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Not useful in this case, but we do need to implement it either way
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame == null) return null;

        // Create a copy of the frame
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Define the lower and upper bounds for the red color range
        Scalar lowerRed1 = new Scalar(0, 100, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 100, 100);
        Scalar upperRed2 = new Scalar(180, 255, 255);

        // Create masks for the red color ranges
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Core.inRange(hsvFrame, lowerRed1, upperRed1, mask1);
        Core.inRange(hsvFrame, lowerRed2, upperRed2, mask2);

        // Combine the masks
        Mat combinedMask = new Mat();
        Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, combinedMask);

        // Fill in the holes
        Imgproc.dilate(combinedMask, combinedMask, new Mat());
        Imgproc.erode(combinedMask, combinedMask, new Mat());

        // Find the rectangles using edge detection
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter out the contours that are not rectangles
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            if (contour == null) continue;
            double perimeter = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), approx, 0.02 * perimeter, true);
            if (approx.toList().size() == 4) {
                contours.set(i, new MatOfPoint(approx.toArray()));
            } else {
                contours.remove(i);
                i--;
            }
        }

        // Find the largest rectangle and draw a rotated bounding box
        if (contours.size() > 0) {
            MatOfPoint largestContour = findLargestContour(contours);
            if (largestContour != null) {
                MatOfPoint2f largestContour2f = new MatOfPoint2f(largestContour.toArray());
                RotatedRect rotatedRect = Imgproc.minAreaRect(largestContour2f);
                Point[] vertices = new Point[4];
                rotatedRect.points(vertices);

                for (int j = 0; j < 4; j++) {
                    Imgproc.line(frame, vertices[j], vertices[(j + 1) % 4], new Scalar(255, 0, 0), 2);
                }

                // Draw the center point
                Point center = rotatedRect.center;
                Imgproc.circle(frame, center, 5, new Scalar(0, 255, 0), -1);

                // Normalize the angle to [-270, 270)
                double angle = rotatedRect.angle;
                if (rotatedRect.size.width < rotatedRect.size.height) {
                    angle = 90 + angle;
                }
                Mat mapMatrix = getRotationMatrix2D(center, angle, 1.0);

                // Display the angle
                String angleText = String.format("Angle: %.2f", angle);
                Imgproc.putText(frame, angleText, new Point(center.x + 10, center.y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);

                // Display the center coordinates
                String centerText = String.format("Center: (%.2f, %.2f)", center.x, center.y);
                Imgproc.putText(frame, centerText, new Point(center.x + 10, center.y + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);

                // Display the corner coordinates
                for (int j = 0; j < 4; j++) {
                    String cornerText = String.format("Corner %d: (%.2f, %.2f)", j, vertices[j].x, vertices[j].y);
                    Imgproc.putText(frame, cornerText, new Point(vertices[j].x, vertices[j].y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
                }
            }
        }

        // Release resources
        hsvFrame.release();
        mask1.release();
        mask2.release();
        combinedMask.release();
        hierarchy.release();
        contours = new ArrayList<>();

        return frame;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        //get the contours from the processFrame method
//        Mat frame = (Mat) userContext;
//        for (int i = 0; i < contours.size(); i++) {
//            MatOfPoint contour = contours.get(i);
//            Rect rect = Imgproc.boundingRect(contour);
//            android.graphics.Rect graphicsRect = makeGraphicsRect(rect, scaleBmpPxToCanvasPx);
//            paint.setColor(Color.RED);
//            paint.setStyle(Paint.Style.STROKE);
//            paint.setStrokeWidth(5);
//            canvas.drawRect(graphicsRect, paint);
//        }


    }
    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }
    }

