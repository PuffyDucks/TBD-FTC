package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "PinchyArm", group = "")
public class pinchyArm extends LinearOpMode{
  private DcMotor ArmMotor;
  private Servo ArmServo;
  private float multiplierWheels = 1;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */

  @Override
  public void runOpMode() {
    //map motors and servos
    ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
    ArmServo = hardwareMap.servo.get("ArmServo");
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
    ((DcMotorEx) ArmMotor).setMotorEnable();
  }

  private void set() {
    // Set direction of devices
    ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
  }

  private void move() {
    // Movement of robot with wheels
    // Left analog stick input
    if(gamepad1.a){
      ArmMotor.setPower(0.5);
    }
    else if(gamepad1.b){
      ArmMotor.setPower(-0.5);
    }
    else {
      ArmMotor.setPower(0);
    }

    if(gamepad1.right_trigger>0.5) {
      ArmServo.setPosition(1);
    } else {
      ArmServo.setPosition(0);
    }
  }

  private void print() {
    // Prints data for debug purposes
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
