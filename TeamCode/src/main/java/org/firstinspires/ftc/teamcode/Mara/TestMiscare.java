package org.firstinspires.ftc.teamcode.Mara;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="testMiscare")
public class TestMiscare extends LinearOpMode {
    DcMotor leftFront,leftBack,rightFront,rightBack;
    @Override
    public void runOpMode() throws InterruptedException {
        leftBack=hardwareMap.get(DcMotor.class,"leftBack");
        leftFront=hardwareMap.get(DcMotor.class,"leftFront");
        rightFront=hardwareMap.get(DcMotor.class,"rightFront");
        rightBack=hardwareMap.get(DcMotor.class,"rightBack");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                leftBack.setPower(0.4);
                leftFront.setPower(0.4);
                rightBack.setPower(0.4);
                rightFront.setPower(0.4);
            } else {
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }
            if(gamepad1.b){
                leftBack.setPower(-0.4);
                leftFront.setPower(-0.4);
                rightBack.setPower(-0.4);
                rightFront.setPower(-0.4);
            } else {
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }
            telemetry.addData("Power",leftBack.getPower());
            telemetry.update();
        }
    }
}
