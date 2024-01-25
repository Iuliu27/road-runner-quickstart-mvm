package org.firstinspires.ftc.teamcode.cameraStuff;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class cameraHW {
    public static Rect rect1 = new Rect(0, 230, 70, 70);
    public static Rect rect2 = new Rect(290, 160, 70, 70);
    public static Rect rect3 = new Rect(565, 230, 70, 70);


     public static int ok;

    public Scalar lowerBlue =new Scalar(106, 100, 50);
    public Scalar upperBlue =new Scalar (230, 255, 255);


    public Scalar lowerRed = new Scalar(100, 100, 100);
    public Scalar uppeerRed = new Scalar(100, 100, 100);

    OpenCvCamera externalCamera;
    static ZoneDetector pipeline;


    public void initTeamPropCamera(String name) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        externalCamera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "WebcamG"), cameraMonitorViewId
        );

        pipeline = new cameraHW.ZoneDetector();
        externalCamera.setPipeline(pipeline);

        /// externalCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        externalCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                externalCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }




    public static class ZoneDetector extends OpenCvPipeline {
        public static Rect rect1 = new Rect(0, 116, 75, 76);
        public  static Rect rect2 = new Rect(160, 102, 75, 76);

        private Mat hsvImage = new Mat();
        private Mat mask = new Mat();
        private Mat hierarchy = new Mat();

        private List<MatOfPoint> contours = new ArrayList<>();
        public static Point squareCenter = new Point();
        public Scalar nonSelectedColor = new Scalar(255, 0, 0);
        public Scalar selectedColor = new Scalar(0, 0, 255);
         public Scalar lowerBlue = new Scalar(106, 165, 55);
         public Scalar upperBlue = new Scalar(230, 255, 255);
        public Scalar lowerRed = new Scalar (0, 90, 7);
        public Scalar upperRed = new Scalar (10, 255, 196);
        private int selectedRect = -1;

        public static String isPointInsideRect() {
            if (rect1.contains(squareCenter)) {
                return "LEFT";
            } else if (rect2.contains(squareCenter)) {
                return "MIDDLE";
            } else {
                return "RIGHT";
            }
        }


        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);


            if (ok==1)
            Core.inRange(hsvImage, lowerRed, upperRed, mask);


            contours.clear();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            Mat processedFrame = input.clone();
            Imgproc.drawContours(processedFrame, contours, -1, new Scalar(0, 255, 0), 2);

            calculateSquarePosition();

            String zone = isPointInsideRect();

            if (!contours.isEmpty()) {
                String coordinates = "X: " + String.format("%.2f", squareCenter.x) + " Y: " + String.format("%.2f", squareCenter.y);
                Imgproc.putText(
                        processedFrame,
                        coordinates,
                        new Point(10, 130),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        new Scalar(0, 0, 255),
                        1
                );
                Imgproc.putText(
                        processedFrame,
                        "Zone: " + zone,
                        new Point(10, 100),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        1,
                        new Scalar(255, 255, 255),
                        2
                );
            }
//            Imgproc.putText(
//                    processedFrame,
//                    "Area",
//                    new Point(30, 150),
//                    Imgproc.FONT_HERSHEY_SIMPLEX,
//                    0.5,
//                    new Scalar(0, 0, 255),
//                    1
//            );

            drawRectangles(processedFrame);

            return processedFrame;
        }

        public Point getSquareCenter() {
            return squareCenter;
        }


        private void drawRectangles(Mat input) {
            Imgproc.rectangle(input, rect1, nonSelectedColor);
            Imgproc.rectangle(input, rect2, nonSelectedColor);


            switch (selectedRect) {
                case 1:
                    Imgproc.rectangle(input, rect1, selectedColor);
                    break;
                case 2:
                    Imgproc.rectangle(input, rect2, selectedColor);
                    break;
            }
        }

        private void calculateSquarePosition() {
            squareCenter.x = 0;
            squareCenter.y = 0;

            if (!contours.isEmpty()) {
                double maxArea = -1;
                int maxAreaIdx = -1;
                for (int i = 0; i < contours.size(); i++) {
                    double area = Imgproc.contourArea(contours.get(i));
                    if (area > maxArea) {
                        maxArea = area;
                        maxAreaIdx = i;
                    }

                }

                if (maxAreaIdx != -1) {
                    // Calculate the center of the largest contour
                    Moments moments = Imgproc.moments(contours.get(maxAreaIdx));
                    squareCenter.x = moments.m10 / moments.m00;
                    squareCenter.y = moments.m01 / moments.m00;
                }
            }
        }
//        private void correctArea(){
//
//            squareCenter.x = 0;
//            squareCenter.y = 0;
//            if(!contours.isEmpty()){
//                double maxArea = -1;
//                int maxAreaC = -1;
//                for(int i=0;i<contours.size();i++){
//                    double area = Imgproc.contourArea(contours.get(maxAreaC));
//                    if()
//                }
//            }
//
//        }
    }
}
