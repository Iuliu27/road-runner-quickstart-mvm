package org.firstinspires.ftc.teamcode.AUTONOMI.RED;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import  com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.Red.AutoRedDreaptaBottom;
import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.DefVal;
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

@Autonomous(name = "Red_Dreapta",group="Iuliu")
public class Red_Dreapta extends LinearOpMode {
    Pose2d beginPose = new Pose2d(11, -58, Math.toRadians(90));
    HardwareMapping robot = new HardwareMapping();
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    OpenCvCamera externalCamera;
    Pose2d cPose;
    Servo intakeServoRight,intakeServoLeft;
    nume pipeline;
    String PropZone ="RIGHT";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        intakeServoLeft=hardwareMap.get(Servo.class,"intakeServoLeft");
        intakeServoRight=hardwareMap.get(Servo.class,"intakeServoRight");
        intakeServoLeft.setPosition(DefVal.iLevel6);
        intakeServoRight.setPosition(DefVal.iLevel6);

        // Initialize the camera and pipeline
        initExternalCamera();

        while (!isStarted() && !isStopRequested())
        {
            PropZone = pipeline.isPointInsideRect();

            if (pipeline.isPointInsideRect()=="LEFT") {
                telemetry.addData("LEFT","");
            } else if(pipeline.isPointInsideRect()=="RIGHT"){
                telemetry.addData("RIGHT","");
            }else if(pipeline.isPointInsideRect()=="MIDDLE") {
                telemetry.addData("MIDDLE","");
            }

            telemetry.update();
        }

        Action LineLeftBack=drive.actionBuilder(beginPose)  //1
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(2,-35,Math.toRadians(120)),Math.toRadians(90))
                .setTangent(Math.toRadians(270)) //-90  ,270,180
                //robotul este in pozitia principala
                .splineToLinearHeading(new Pose2d(11,-56,Math.toRadians(90)),Math.toRadians(-90))
                .build();

        Action LineMiddleBack = drive.actionBuilder(beginPose) //mijloc  1
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(11,-30,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(270)) //-90  ,270,180
                //robotul este in pozitia principala
                .splineToLinearHeading(new Pose2d(11,-56,Math.toRadians(90)),Math.toRadians(-90))
                .build();

        Action LineRightBack=drive.actionBuilder(beginPose) //dreapta  1
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(17,-35,Math.toRadians(60)),Math.toRadians(90))
                .setTangent(Math.toRadians(270)) //-90  ,270,180
                //robotul este in pozitia principala
                .splineToLinearHeading(new Pose2d(11,-56,Math.toRadians(90)),Math.toRadians(-90))
                .build();

        cPose= new Pose2d(11,-56,Math.toRadians(90));

        Action BeginToBackboardLeft=drive.actionBuilder(cPose) //1
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(45.5,-25,Math.toRadians(0)),Math.toRadians(90))
                .build();

        cPose= new Pose2d(11,-56,Math.toRadians(90));

        Action BeginToBackboardMiddle=drive.actionBuilder(cPose)   //1
                .setTangent(Math.toRadians(90))
                //begin backboard
                .splineToLinearHeading(new Pose2d(45.5,-28,Math.toRadians(0)),Math.toRadians(90))
                .build();

        cPose= new Pose2d(11,-56,Math.toRadians(90));

        Action BeginToBackboardRight=drive.actionBuilder(cPose)  //1
                .setTangent(Math.toRadians(90))
                //begin backboard
                .splineToLinearHeading(new Pose2d(45.5,-36,Math.toRadians(0)),Math.toRadians(90))
                .build();

        cPose=new Pose2d(45.5,-28,Math.toRadians(0));

        Action ToStack=drive.actionBuilder(cPose)  //1
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(31.8,-63,Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-30,-64,Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-64.5,-34,Math.toRadians(0)),Math.toRadians(180))
                .afterDisp(5,new SequentialAction(
                        new ParallelAction(
                                intake.reversePixel(),
                                intake.angle(5)
                        ),
                        //new SleepAction(1.5),
                        new ParallelAction(
                                outtake.bottomHook("closed"),outtake.upperHook("closed")
                        ),
                        intake.stop()
                ))
                .build();

        cPose=new Pose2d(-63,-37,Math.toRadians(0));

        Action ToBackboardRight=drive.actionBuilder(cPose) //1
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-10,-58,Math.toRadians(0)),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20,-56,Math.toRadians(0)),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45.5,-36,Math.toRadians(0)),Math.toRadians(0))
                .build();

        cPose=new Pose2d(50,-36,Math.toRadians(0));

        Action Parking=drive.actionBuilder(cPose)  //1
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(51,-58,Math.toRadians(0)),Math.toRadians(90))
                .build();

        waitForStart();

        externalCamera.stopStreaming();
        externalCamera.closeCameraDevice();

        if(PropZone=="LEFT"){
            Actions.runBlocking(new SequentialAction(
                    LineLeftBack,
                    new SequentialAction(
                            new ParallelAction(
                                    BeginToBackboardLeft,
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("autonom")
                                    )
                            ),
                            new SleepAction(0.5),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    ToStack,
                    ToBackboardRight,
                    new SequentialAction(
                            new ParallelAction(
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("low")
                                    )
                            ),
                            new SleepAction(1.2),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(1.2),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    Parking
            ));
        } else if(PropZone=="MIDDLE") {
            Actions.runBlocking(new SequentialAction(
                    LineMiddleBack,
                    new SequentialAction(
                            new ParallelAction(
                                    BeginToBackboardMiddle,
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("autonom")
                                    )
                            ),
                            //new SleepAction(0.5),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(0.7),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    ToStack,
                    ToBackboardRight,
                    new SequentialAction(
                            new ParallelAction(
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("low")
                                    )
                            ),
                            new SleepAction(1.2),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(1.2),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    Parking
            ));
        } else if(PropZone=="RIGHT") {
            Actions.runBlocking(new SequentialAction(
                    LineRightBack,
                    new SequentialAction(
                            new ParallelAction(
                                    BeginToBackboardRight,
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("autonom")
                                    )
                            ),
                            //new SleepAction(0.5),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(0.7),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    ToStack,
                    ToBackboardRight,
                    new SequentialAction(
                            new ParallelAction(
                                    new ParallelAction(
                                            outtake.pivot(DefVal.pivot60),
                                            outtake.roll(DefVal.roll60),
                                            outtake.runToPosition("low")
                                    )
                            ),
                            new SleepAction(1.2),
                            new ParallelAction(
                                    outtake.upperHook("open"),
                                    outtake.bottomHook("open")
                            ),
                            new SleepAction(1.2),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot0),
                                    outtake.roll(DefVal.roll0)
                            ),
                            new SleepAction(1),
                            outtake.runToPosition("ground")
                    ),
                    Parking
            ));
        }
        while (opModeIsActive()) {
            //telemetry
        }
    }






//


    private void initExternalCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        externalCamera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "WebcamG"), cameraMonitorViewId
        );

        pipeline = new nume();
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

    public class nume extends OpenCvPipeline {
        public Rect rect1 = new Rect(0, 116, 75, 76);
        public Rect rect2 = new Rect(160, 102, 75, 76);

        private Mat hsvImage = new Mat();
        private Mat mask = new Mat();
        private Mat hierarchy = new Mat();

        private List<MatOfPoint> contours = new ArrayList<>();
        private Point squareCenter = new Point();
        public Scalar nonSelectedColor = new Scalar(255, 0, 0);
        public Scalar selectedColor = new Scalar(0, 0, 255);
        // public Scalar lowerBlue = new Scalar(106, 165, 55);
        // public Scalar upperBlue = new Scalar(230, 255, 255);
        public Scalar lowerRed = new Scalar (0, 90, 7);
        public Scalar upperRed = new Scalar (10, 255, 196);
        private int selectedRect = -1;

        public String isPointInsideRect() {
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

//        private void calculateSquarePosition() {
//            squareCenter.x = 0;
//            squareCenter.y = 0;
//
//            if (!contours.isEmpty()) {
//                double maxArea = -1;
//                int maxAreaIdx = -1;
//                for (int i = 0; i < contours.size(); i++) {
//                    double area = Imgproc.contourArea(contours.get(i));
//                    if (area > maxArea) {
//                        maxArea = area;
//                        maxAreaIdx = i;
//                    }
//
//                }
//
//                if (maxAreaIdx != -1) {
//                    // Calculate the center of the largest contour
//                    Moments moments = Imgproc.moments(contours.get(maxAreaIdx));
//                    squareCenter.x = moments.m10 / moments.m00;
//                    squareCenter.y = moments.m01 / moments.m00;
//                }
//            }
//        }
private void calculateSquarePosition() {
    squareCenter.x = 0;
    squareCenter.y = 0;

    if (!contours.isEmpty()) {
        double maxArea = -1;
        int maxAreaIdx = -1;
        double minContourArea =  19;

        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area >= minContourArea && area > maxArea) {
                maxArea = area;
                maxAreaIdx = i;
            }
        }

        if (maxAreaIdx != -1) {
            Moments moments = Imgproc.moments(contours.get(maxAreaIdx));
            squareCenter.x = moments.m10 / moments.m00;
            squareCenter.y = moments.m01 / moments.m00;
        }
    }
}

    }
}
