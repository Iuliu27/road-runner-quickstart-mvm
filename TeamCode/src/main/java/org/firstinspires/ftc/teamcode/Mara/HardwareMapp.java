package org.firstinspires.ftc.teamcode.Mara;

import static org.firstinspires.ftc.teamcode.Variables.DefVal.pivot0;
import static org.firstinspires.ftc.teamcode.Variables.DefVal.pivot60;
import static org.firstinspires.ftc.teamcode.Variables.DefVal.roll0;
import static org.firstinspires.ftc.teamcode.Variables.DefVal.roll60;
//import static org.firstinspires.ftc.teamcode.Variables.DefVal.yaw0;
//import static org.firstinspires.ftc.teamcode.Variables.DefVal.yaw90;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.Variables.DefVal;
import org.opencv.core.Scalar;

public class HardwareMapp {

    /*Scriu aici ce mai trebuie facut:
     * Prindere pixeli(hook)-gata(nush exact daca trebuie sa verific in ColorDetected)
     * Sa fac sa lumineze LED-urile-gata(mai trebuie blinking pentru alb)
     * Implementare senzori de culoare*/

    double PI = 3.1415;
    double GEAR_MOTOR_GOBILDA_312_TICKS = 537.7;
    double WHEEL_DIAMETER_CM = 3.565;
    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA_312_TICKS / (WHEEL_DIAMETER_CM * PI);
    public GamepadEx gamepad1Ex,gamepad2Ex;

    public void gamepadInit(Gamepad gamepad1, Gamepad gamepad2){
        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
    }

    public enum LEDColor{
        Purple, //red
        Green,  //green
        Yellow, //between red & green
        White, //blinking
        None //none
    }
    public DcMotorEx hangMotor;  //motor pentru ridicat robotul in hang
    public DcMotorEx misumMotorLeft;  //motor pentru misum-ul stang
    public DcMotorEx misumMotorRight;  //motor pentru misum-ul drept
    public DcMotorEx intakeMotor;  //motor pentru maturice           //motoare

    public CRServo intakeCRServo;  //servo CR pentru intake
    public Servo outtakeDoorServo;  //servo pentru deschis outake-ul
    public Servo planeServo; //servo pentru avion
    public Servo servoBottomHook;
    public Servo servoUpperHook;
    public Servo intakeServoLeft;
    public Servo intakeServoRight;
    public Servo OuttakeTurn90Servo;
    public Servo backboardAlignServoLeft;
    public Servo backboardAlignServoRight;  //servo-uri pentru pus outtaku-ul la 60 de grade(backboard)
    public Servo turnOuttakeLeft;
    public Servo turnOuttakeRight;

    public SensorColor SensorfirstHook;  //senzor pentru primul pixel
    public SensorColor SensorsecondHook;  //senzor pentru al doilea pixel

    public DigitalChannel LEDdowngreen;
    public DigitalChannel LEDupgreen;
    public DigitalChannel LEDdownred;
    public DigitalChannel LEDupred;

    BNO055IMU imu;
    HardwareMap HW=null;
    public HardwareMapp(){}

    public void init(HardwareMap hardwareMap) {

        //HW=hw;

        hangMotor=HW.get(DcMotorEx.class,"hangMotor");
        misumMotorLeft=HW.get(DcMotorEx.class,"misumMotorLeft");
        misumMotorRight=HW.get(DcMotorEx.class,"misumMotorRight");
        intakeMotor=HW.get(DcMotorEx.class,"intakeMotor");

        intakeCRServo=HW.get(CRServo.class,"intakeServo");
        outtakeDoorServo=HW.get(Servo.class,"outtakeDoorServo");
        planeServo=HW.get(Servo.class,"planeServo");
        servoBottomHook=HW.get(Servo.class,"servoBottomHook");
        servoUpperHook=HW.get(Servo.class,"servoUpperHook");
        intakeServoLeft=HW.get(Servo.class,"intakeServoLeft");
        intakeServoRight=HW.get(Servo.class,"intakeServoRight");
        OuttakeTurn90Servo=HW.get(Servo.class,"OuttakeTurn90Servo");
        backboardAlignServoLeft=HW.get(Servo.class,"backboardAlignServoLeft");
        backboardAlignServoRight=HW.get(Servo.class,"backboardAlignServoRight");
        turnOuttakeLeft=HW.get(Servo.class,"turnOuttakeLeft");
        turnOuttakeRight=HW.get(Servo.class,"turnOuttakeRight");

        SensorfirstHook=HW.get(SensorColor.class,"firstHookPixel");
        SensorsecondHook=HW.get(SensorColor.class,"secondHookPixel");

        LEDdowngreen=HW.get(DigitalChannel.class,"LEDdownGreen");
        LEDdownred=HW.get(DigitalChannel.class,"LEDdownRed");
        LEDupgreen=HW.get(DigitalChannel.class,"LEDupGreen");
        LEDupred=HW.get(DigitalChannel.class,"LEDupRed");

        //imu=HW.get(BNO055IMU.class,"imu");
    }

    public Action launchPlane(){     //actiune pentru decolarea avionului
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                planeServo.setPosition(0.2);
                return false;
            }
        };
    }
    public Action intakeRoller(String stare){     //actiune pentru intake (roller)
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "in":
                        intakeCRServo.setPower(DefVal.intakeRollerPower);
                        break;
                    case "out":
                        intakeCRServo.setPower(-DefVal.intakeRollerPower);
                        break;
                    case "off":
                        intakeCRServo.setPower(0);
                        break;
                }
                return false;
            }
        };
    }
    public Action outtakeRoll(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "ground":
                        backboardAlignServoLeft.setPosition(roll0);
                        backboardAlignServoRight.setPosition(roll0);
                    case "backboard":
                        backboardAlignServoLeft.setPosition(roll60);
                        backboardAlignServoRight.setPosition(roll60);
                }
                return false;
            }
        };
    }
    public Action turnOuttakeUp_Down(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "up":
                        turnOuttakeLeft.setPosition(pivot60);
                        turnOuttakeRight.setPosition(pivot60);
                    case "down":
                        turnOuttakeLeft.setPosition(pivot0);
                        turnOuttakeRight.setPosition(pivot0);
                }
                return false;
            }
        };
    }
    public Action hang(String stare){     //actiune pentru hang
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "up":
                        hangMotor.setPower(1);
                        CommandScheduler.getInstance().schedule(new WaitCommand(1500));
                        hangMotor.setPower(0);
                        break;
                    case "hang":
                        hangMotor.setPower(0.2);
                        break;
                }
                return false;
            }
        };
    }

    /*public Action openOuttake(String stare){ //deschis/inchis cutie outake
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "open":
                        outtakeDoorServo.setPosition(0.5);
                        break;
                    case "close":
                        outtakeDoorServo.setPosition(0);
                        break;
                }
                return false;
            }
        };
    }*/

    /*public Action turn90Outtake(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "noTurn":
                        //OuttakeTurn90Servo.setPosition(yaw0);
                        break;
                    case "turn":
                        //OuttakeTurn90Servo.setPosition(yaw90);
                        break;
                }
                return false;
            }
        };
    }*/

    public Action maturiceOpen_Close(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare) {
                    case "in":
                        intakeMotor.setPower(DefVal.intakeMotorPower);
                        break;
                    case "out":
                        intakeMotor.setPower(-DefVal.intakeMotorPower);
                        break;
                    case "off":
                        intakeMotor.setPower(0);
                        break;
                }
                return false;
            }
        };
    }

    public Action maturiceLevel(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "Level1":
                        intakeServoLeft.setPosition(DefVal.iLevel1);
                        intakeServoRight.setPosition(DefVal.iLevel1);
                    case "Level2":
                        intakeServoLeft.setPosition(DefVal.iLevel2);
                        intakeServoRight.setPosition(DefVal.iLevel2);
                    case "Level3":
                        intakeServoLeft.setPosition(DefVal.iLevel3);
                        intakeServoRight.setPosition(DefVal.iLevel3);
                    case "Level4":
                        intakeServoLeft.setPosition(DefVal.iLevel4);
                        intakeServoRight.setPosition(DefVal.iLevel4);
                    case "Level5":
                        intakeServoLeft.setPosition(DefVal.iLevel5);
                        intakeServoRight.setPosition(DefVal.iLevel5);
                    case "Level6":
                        intakeServoLeft.setPosition(DefVal.iLevel6);
                        intakeServoRight.setPosition(DefVal.iLevel6);
                }
                return false;
            }
        };
    }
    /*float[] hsvValues = new float[3];
    Scalar detectedColorHSV = new Scalar(hsvValues[0], hsvValues[1], hsvValues[2]);

    public static class ColorRange{
        public Scalar[] greenColorRange = {
                new Scalar(109, 45, 1), // Valoare minimă HSV pentru verde
                new Scalar(120, 58, 16) // Valoare maximă HSV pentru verde
        };
        public Scalar[] yellowColorRange={
                new Scalar(35,54,2), //Valoare minima HSV pentru galben
                new Scalar(64,64,22) //Valoare maxima HSV pentru galben
        };
        public Scalar[] purpleColorRange={
                new Scalar(27,18,3), //Valoare minima HSV pentru mov
                new Scalar(249,27,23) //Valoare maxima HSV pentru mov
        };
        public Scalar[] whiteColorRange={
                new Scalar(92,8,4), //Valoare minima HSV pentru alb
                new Scalar(135,24,41) //Valoare maxima HSV pentru alb
        };
    }*/
    /*public LEDColor ColorDetected(Scalar targetColor, Scalar[] colorRange){
        ColorRange colorRangeDet=new ColorRange();
        if(detectedColorHSV.val[0] >= colorRangeDet.greenColorRange[0].val[0] && detectedColorHSV.val[0] <= colorRangeDet.greenColorRange[1].val[0]){
            //vede culoare verde
            return LEDColor.Green;
        }
        if(detectedColorHSV.val[0] >= colorRangeDet.yellowColorRange[0].val[0] && detectedColorHSV.val[0] <= colorRangeDet.yellowColorRange[1].val[0]){
            //vede culoaregalben
            return LEDColor.Yellow;
        }
        if(detectedColorHSV.val[0] >= colorRangeDet.whiteColorRange[0].val[0] && detectedColorHSV.val[0] <= colorRangeDet.whiteColorRange[1].val[0]){
            //vede culoare alb
            return LEDColor.White;
        }
        if(detectedColorHSV.val[0] >= colorRangeDet.purpleColorRange[0].val[0] && detectedColorHSV.val[0] <= colorRangeDet.purpleColorRange[1].val[0]){
            //vede culoare mov
            return LEDColor.Purple;
        }
        return LEDColor.None;
    }*/

    /*public Action LED(String led,String color){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                DigitalChannel LED1=null;
                DigitalChannel LED2=null;
                if(led.equals("up")){
                    LED1=LEDupgreen;
                    LED2=LEDupred;
                }
                if(led.equals("down")){
                    LED1=LEDdowngreen;
                    LED2=LEDdownred;
                }
                LEDforDrivers(color,LED1,LED2);
                return false;
            }
        };
    }*/

    /*public Action LEDforDrivers(String stare,DigitalChannel LED1,DigitalChannel LED2){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare) {
                    case "NONE":
                        LED1.setState(false); //LED1=green,LED2=red
                        LED2.setState(false);
                    case "GREEN":
                        LED1.setState(true);
                        LED2.setState(false);
                    case "PURPLE":
                        LED1.setState(false);
                        LED2.setState(true);
                    case "YELLOW":
                        LED1.setState(true);
                        LED2.setState(true);
                    case "WHITE":
                        Actions.runBlocking(WhitePixelBlinking(LED1,LED2)); //trebuie facuta actiunea de blinking
                }
                return false;
            }
        };
    }*/

    /*public Action WhitePixelBlinking(DigitalChannel LED1, DigitalChannel LED2){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //cod pentru blinking
                try {
                    LED1.setState(true);LED2.setState(false);
                    Thread.sleep(500);
                    LED2.setState(true);LED1.setState(false);
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                return true;
            }
        };
    }*/

    public Action misumHeight(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                misumMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                misumMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                switch (stare){
                    case "GROUND":
                        misumMotorLeft.setTargetPosition((int) (DefVal.LiftGROUND*TICKS_PER_CM_Z));
                        misumMotorRight.setTargetPosition((int) (DefVal.LiftGROUND*TICKS_PER_CM_Z));
                    case "LOW":
                        misumMotorLeft.setTargetPosition((int) (DefVal.LiftLOW*TICKS_PER_CM_Z));
                        misumMotorRight.setTargetPosition((int) (DefVal.LiftLOW*TICKS_PER_CM_Z));
                    case "MIDDLE":
                        misumMotorLeft.setTargetPosition((int) (DefVal.LiftMIDDLE*TICKS_PER_CM_Z));
                        misumMotorRight.setTargetPosition((int) (DefVal.LiftMIDDLE*TICKS_PER_CM_Z));
                    case "HIGH":
                        misumMotorLeft.setTargetPosition((int) (DefVal.LiftHIGH*TICKS_PER_CM_Z));
                        misumMotorRight.setTargetPosition((int) (DefVal.LiftHIGH*TICKS_PER_CM_Z));
                }
                misumMotorLeft.setPower(1);
                misumMotorRight.setPower(1);
                return false;
            }
        };
    }

    public Action bottomHook(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "close":
                        servoBottomHook.setPosition(DefVal.bottomHookClosed);
                    case "open":
                        servoBottomHook.setPosition(DefVal.bottomHookOpen);
                }
                return false;
            }
        };
    }

    public Action upperHook(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "close":
                        servoUpperHook.setPosition(DefVal.upperHookClosed);
                    case "open":
                        servoUpperHook.setPosition(DefVal.upperHookOpen);
                }
                return false;
            }
        };
    }

    /*public class imu{
        public double TILT_THRESHOLD = 20;
        public double ACCEL_THRESHOLD = 20;  //am pus valori cam random. Trebuie sa ma documentez
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration accel = imu.getLinearAcceleration();

        double rollAngle = angles.secondAngle;
        boolean isTilted = Math.abs(rollAngle) > TILT_THRESHOLD;

        double lateralAcceleration = accel.yAccel;
        boolean hasLateralAcceleration = Math.abs(lateralAcceleration) > ACCEL_THRESHOLD;

    }*/
}