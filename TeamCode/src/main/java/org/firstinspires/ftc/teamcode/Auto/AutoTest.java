package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.signature.qual.FieldDescriptorForPrimitiveOrArrayInUnnamedPackage;
import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseTransfer;

@Autonomous(name="VASILEEEEE")
public class AutoTest extends LinearOpMode {
    HardwareMapping Robot=new HardwareMapping();
    HardwareMapping.Intake intake = Robot.new Intake();
    HardwareMapping.Outtake outtake = Robot.new Outtake();
    private MecanumDrive drive;
    private Pose2d cPose;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap);
        Pose2d firstPose=new Pose2d(-34.5,-58,90);
        Pose2d RightLane=new Pose2d(-29,-33,60);

        cPose=firstPose;

        Action TrajRightLane = drive.actionBuilder(cPose)  //Traiectorie pana la linia din dreapta
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-29, -33, Math.toRadians(60)), Math.toRadians(60))
                .build();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            Actions.runBlocking(new SequentialAction(
                    TrajRightLane
            ));
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
