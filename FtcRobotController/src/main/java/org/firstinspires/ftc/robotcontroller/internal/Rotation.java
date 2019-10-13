package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Rotation", group = "")
public class Rotation extends LinearOpMode{
    //Driver();
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor LeftBack;
    private DcMotor RightBack;
    private double setAPower;
    private double setBPower;
    private double magnitude;
    private double highestPower;
    private double adjuster;
    @Override
    public void runOpMode(){
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        call();
        set();
        while(opModeIsActive()){
            move();
            //print();
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
        RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    private void move() {
        // Movement of robot with wheels
        // Left analog stick input
        // X and Y values for omni wheel sets (Top Left, Bottom Right) and (Top Right, Bottom Left)
        // Values can go over 1
        setAPower = gamepad1.left_stick_x + gamepad1.left_stick_y;
        setBPower = gamepad1.left_stick_y - gamepad1.left_stick_x;
        // Magnitude of joystick
        magnitude = Math.sqrt(Math.pow(setAPower, 2) + Math.pow(setBPower, 2));
        // Highest value for wheel power
        highestPower = Math.abs(setAPower) + Math.abs(setBPower);
        // Value to make numbers not go over 1
        adjuster = magnitude / highestPower;
        // Power values for omni wheel sets, adjusted to not go over 1 
        LeftFront.setPower(setAPower * adjuster * 0.5 + gamepad1.right_stick_x * 0.5);
        RightBack.setPower(setAPower * adjuster * 0.5 - gamepad1.right_stick_x * 0.5);
        RightFront.setPower(setBPower * adjuster * 0.5 - gamepad1.right_stick_x * 0.5);
        LeftBack.setPower(setBPower * adjuster * 0.5 + gamepad1.right_stick_x * 0.5);
    }
}
