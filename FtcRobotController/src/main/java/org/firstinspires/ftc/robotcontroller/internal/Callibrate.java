package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Callibrate", group = "")
public class Callibrate extends LinearOpMode{
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightFront;
    private DcMotor RightBack;

    private double multiplier = 1;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {
        //map motors and servos
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        //set encoder modes

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //these are functions below
        call();
        set();

        while(opModeIsActive()) {
            //see below
            move();
            print();
        }
    }

    private void call() {
        // Call program, and devices
        waitForStart();
        ((DcMotorEx) LeftFront).setMotorEnable();
        ((DcMotorEx) LeftBack).setMotorEnable();
        ((DcMotorEx) RightFront).setMotorEnable();
        ((DcMotorEx) RightBack).setMotorEnable();
    }

    private void set() {
        // Set direction of devices
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void move() {
        if(gamepad1.x) LeftFront.setPower(1 * multiplier);
        else if(gamepad1.y) LeftBack.setPower(1 * multiplier);
        else if(gamepad1.a) RightFront.setPower(1 * multiplier);
        else if(gamepad1.b) RightBack.setPower(1 * multiplier);
        else if(gamepad1.left_bumper){
            LeftBack.setPower(1);
            LeftFront.setPower(-1);
            RightBack.setPower(-1);
            RightFront.setPower(1);
        }
        else if(gamepad1.right_bumper) {
            LeftBack.setPower(-1);
            LeftFront.setPower(1);
            RightBack.setPower(1);
            RightFront.setPower(-1);
        }
        else {
            LeftBack.setPower(0);
            LeftFront.setPower(0);
            RightBack.setPower(0);
            RightFront.setPower(0);
        }
    }

    private void print() {
        // Prints data for debug purposes
        telemetry.addData("Encode LB", LeftBack.getCurrentPosition());
        telemetry.addData("Encode LF", LeftFront.getCurrentPosition());
        telemetry.addData("Encode RF", RightFront.getCurrentPosition());
        telemetry.addData("Encode RB", RightBack.getCurrentPosition());

        telemetry.addData("LB", LeftBack.getPower());
        telemetry.addData("RB", RightBack.getPower());
        telemetry.addData("LF", LeftFront.getPower());
        telemetry.addData("RF", RightFront.getPower());

        telemetry.update();
    }
}