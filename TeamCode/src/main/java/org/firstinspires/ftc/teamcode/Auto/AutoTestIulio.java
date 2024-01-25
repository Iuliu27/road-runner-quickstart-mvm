package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.DefVal;

@Autonomous(name="Vasilica")
public class AutoTestIulio extends LinearOpMode {
    Pose2d beginPose = new Pose2d(-34.5, -58, Math.toRadians(90));
    HardwareMapping robot = new HardwareMapping();
    Servo intakeServoRight,intakeServoLeft;
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    Pose2d curentPose;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        intakeServoRight=hardwareMap.get(Servo.class,"intakeServoRight");
        intakeServoLeft=hardwareMap.get(Servo.class,"intakeServoLeft");
        intakeServoLeft.setPosition(DefVal.iLevel6);
        intakeServoRight.setPosition(DefVal.iLevel6);

        Action blonda=drive.actionBuilder(beginPose) //dreapta
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-27, -32.5, Math.toRadians(60)), Math.toRadians(60))
                .build();

        //beginPose=new Pose2d(-34.5, -58, Math.toRadians(90));

        Action iulio = drive.actionBuilder(beginPose) //mijloc
                .splineToLinearHeading(new Pose2d(-35,-29,Math.toRadians(90)),Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-35,-42,Math.toRadians(90)),Math.toRadians(90))
                .build();

        //beginPose=new Pose2d(-34.5, -58, Math.toRadians(90));

        Action busca= drive.actionBuilder(beginPose) //stanga
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-39,-36,Math.toRadians(120)),Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-37,-45,Math.toRadians(120)),Math.toRadians(90))
                .build();

        curentPose=new Pose2d(-37,-45,Math.toRadians(90));

        waitForStart();
        Actions.runBlocking(
                //iulio
                blonda
                //busca
        );

    }
}
