package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Driver", group = "")
public class Driver extends LinearOpMode{
  private DcMotor LeftFront;
  private DcMotor LeftBack;
  private DcMotor RightFront;
  private DcMotor RightBack;
  private DcMotor ClawArm;
  private DcMotor ClawArm2;
  /**
  * This function is executed when this Op Mode is selected from the Driver Station.
  */

  @Override
  public void runOpMode() {
    LeftFront = hardwareMap.dcMotor.get("LeftFront");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightFront = hardwareMap.dcMotor.get("RightFront");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    // ClawArm = hardwareMap.dcMotor.get("ClawArm");
    // ClawArm2 = hardwareMap.dcMotor.get("ClawArm2");

    //these are functions below
    call();
    set();

    while(opModeIsActive()){
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
    // ((DcMotorEx) ClawArm).setMotorEnable();
    // ((DcMotorEx) ClawArm2).setMotorEnable();
  }

  private void set() {
    // Set direction of devices
    LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
    RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    // ClawArm.setDirection(DcMotorSimple.Direction.FORWARD);
    // ClawArm2.setDirection(DcMotorSimple.Direction.FORWARD);
  }

  private void move() {
    // Movement of robot with wheels
    // Left analog stick input
    LeftFront.setPower(gamepad1.left_stick_y * 0.5 - gamepad1.right_stick_x * 0.5);
    LeftBack.setPower(gamepad1.left_stick_y * 0.5 - gamepad1.right_stick_x * 0.5);
    RightFront.setPower(gamepad1.left_stick_y * 0.5 + gamepad1.right_stick_x * 0.5);
    RightBack.setPower(gamepad1.left_stick_y * 0.5 + gamepad1.right_stick_x * 0.5);
    
    // if(gamepad1.a){
    //   ClawArm.setPower(0.3);
    // }
    // else if(gamepad1.b){
    //   ClawArm.setPower(-0.3);
    // }
    // else{
    //   ClawArm.setPower(0);
    // }
    
    // if(gamepad1.x){
    //   ClawArm2.setPower(0.3);
    // }
    // else if(gamepad1.y){
    //   ClawArm2.setPower(-0.3);
    // }
    // else{
    //   ClawArm2.setPower(0);
    // }
  }

  private void print() {
    // Prints data for debug purposes
    telemetry.addData("joystick1 x", gamepad1.left_stick_x);
    // telemetry.addData("Left Wheel Power", LeftWheel.getPower());
    // telemetry.addData("Right Wheel Power", RightWheel.getPower());
    //telemetry.addData("Left Wheel Position", LeftWheel.getCurrentPosition());
    //telemetry.addData("Right Wheel Position", RightWheel.getCurrentPosition());
    //telemetry.addData("GamePad2 Left Stick Y", gamepad2.left_stick_y);
    //telemetry.addData("Claw 1 Power", ClawArm.getPower());
    //telemetry.addData("Claw 2 Power", ClawArm2.getPower());
    telemetry.update();
  }
}
