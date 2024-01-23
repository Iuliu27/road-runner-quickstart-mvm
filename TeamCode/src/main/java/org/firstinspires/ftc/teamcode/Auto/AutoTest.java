package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;


import org.checkerframework.checker.signature.qual.FieldDescriptorForPrimitiveOrArrayInUnnamedPackage;
import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseTransfer;
import org.opencv.core.Mat;

@Autonomous(name="VASILEEEEE")
public class AutoTest extends LinearOpMode { //ok
    HardwareMapping Robot=new HardwareMapping();
    HardwareMapping.Intake intake = Robot.new Intake();
    HardwareMapping.Outtake outtake = Robot.new Outtake();
    private MecanumDrive drive;
    private Pose2d cPose=new Pose2d(-34.5,-58, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Pose2d firstPose=new Pose2d(-34.5,-58,Math.toRadians(60));
        Pose2d RightLane=new Pose2d(-29,-33,Math.toRadians(60));
        //TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(cPose);


        cPose=firstPose;

        Action TrajRightLane = drive.actionBuilder(cPose)  //Traiectorie pana la linia din dreapta
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-29, -33, Math.toRadians(60)), Math.toRadians(60))
                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                TrajRightLane
        ));

        while(opModeIsActive() && !isStopRequested()){
            Pose2d currentPose = drive.pose;
             PoseTransfer.currentPose = currentPose;
            telemetry.addData("x :",drive.pose.position.x);
            telemetry.addData("y :",drive.pose.position.y);
            telemetry.addData("Current Pose :",cPose);
            telemetry.update();
            drive.updatePoseEstimate();
        }
    }
}
