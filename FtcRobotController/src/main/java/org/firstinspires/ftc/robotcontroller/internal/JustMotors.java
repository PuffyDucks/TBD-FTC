package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "JustMotors", group = "")
public class JustMotors extends LinearOpMode{
  private DcMotor LeftFront;
  private DcMotor LeftBack;
  private DcMotor RightFront;
  private DcMotor RightBack;

  private float multiplierWheels = 1;
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

//    //set encoder modes
//    LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
  }

  private void move() {
    // Movement of robot with wheels
    // Left analog stick input
    /*LeftFront.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) * 0.5 - gamepad1.right_stick_x * 0.3)*multiplierWheels);
    LeftBack.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) * 0.5 - gamepad1.right_stick_x * 0.3)*multiplierWheels);
    RightFront.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) * 0.5 + gamepad1.right_stick_x * 0.3)*multiplierWheels);
    RightBack.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) * 0.5 + gamepad1.right_stick_x * 0.3)*multiplierWheels);
    if(gamepad1.right_trigger>0.5) {
      multiplierWheels = 2;
    } else {
      multiplierWheels = 1;
    }*/
    if(gamepad1.x){
      LeftFront.setPower(0.2);
    }
    else if(gamepad1.y){
      RightFront.setPower(0.2);
    }
    else if(gamepad1.b){
      RightBack.setPower(0.2);
    }
    else if(gamepad1.a){
      LeftBack.setPower(0.2);
    }
    else{
      LeftBack.setPower(0);
      LeftFront.setPower(0);
      RightFront.setPower(0);
      RightBack.setPower(0);
    }
  }

  private void print() {
    // Prints data for debug purposes
    telemetry.addData("Encode LB", LeftBack.getCurrentPosition());
    telemetry.addData("Encode LF", LeftFront.getCurrentPosition());
    telemetry.addData("Encode RF", RightFront.getCurrentPosition());
    telemetry.addData("Encode RB", RightBack.getCurrentPosition());

    telemetry.addData("LF", LeftFront.getPower());
    telemetry.addData("RF", RightFront.getPower());
    telemetry.addData("LB", LeftBack.getPower());
    telemetry.addData("RB", RightBack.getPower());
//     telemetry.addData("Left Wheel Power", LeftWheel.getPower());
//     telemetry.addData("Right Wheel Power", RightWheel.getPower());
//    telemetry.addData("Left Wheel Position", LeftWheel.getCurrentPosition());
//    telemetry.addData("Right Wheel Position", RightWheel.getCurrentPosition());
//    telemetry.addData("GamePad2 Left Stick Y", gamepad2.left_stick_y);
//    telemetry.addData("Claw 1 Power", ClawArm.getPower());
//    telemetry.addData("Claw 2 Power", ClawArm2.getPower());
    telemetry.update();
  }
}
