package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Vasilica")
public class AutoTestIulio extends LinearOpMode {
    Pose2d beginPose = new Pose2d(-34.5, -58, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Action blonda=drive.actionBuilder(beginPose) //dreapta
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-27, -32.5, Math.toRadians(60)), Math.toRadians(60))
                .build();

        beginPose=new Pose2d(-34.5, -58, Math.toRadians(90));

        Action iulio = drive.actionBuilder(beginPose) //mijloc
                .splineToLinearHeading(new Pose2d(-35,-29,Math.toRadians(90)),Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-35,-42,Math.toRadians(90)),Math.toRadians(90))
                .build();

        beginPose=new Pose2d(-34.5, -58, Math.toRadians(90));

        Action busca= drive.actionBuilder(beginPose) //stanga
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-39,-36,Math.toRadians(120)),Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-37,-45,Math.toRadians(90)),Math.toRadians(90))
                .setReversed(false)
                .build();

        waitForStart();

        Actions.runBlocking(
                //iulio
                blonda
                //busca
        );

    }
}
