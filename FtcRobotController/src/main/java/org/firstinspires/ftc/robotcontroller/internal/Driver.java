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
  private Servo servo;
  private Servo servo2;

  private float multiplier = 1;

  private double leftFront = 1;
  private double rightFront = 1;
  private double leftBack = 1;
  private double rightBack = 1;
  /**
  * This function is executed when this Op Mode is selected from the Driver Station.
  */

  @Override
  public void runOpMode() {
    LeftFront = hardwareMap.dcMotor.get("LeftFront");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightFront = hardwareMap.dcMotor.get("RightFront");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    ClawArm = hardwareMap.dcMotor.get("ClawArm");
    ClawArm2 = hardwareMap.dcMotor.get("ClawArm2");
    servo = hardwareMap.get(Servo.class, "Grab");
    servo2 = hardwareMap.get(Servo.class, "Grab2");

    //these are functions below
    call();
    set();

    while(opModeIsActive()) {
      //see below
      move();
      callibrate();
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
    ((DcMotorEx) ClawArm).setMotorEnable();
    ((DcMotorEx) ClawArm2).setMotorEnable();
  }

  private void set() {
    // Set direction of devices
    LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
    RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    ClawArm.setDirection(DcMotorSimple.Direction.FORWARD);
    ClawArm2.setDirection(DcMotorSimple.Direction.FORWARD);
  }

  private void move() {
    // Movement of robot with wheels
    // Left analog stick input
    LeftFront.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) * 0.5 - gamepad1.right_stick_x * 0.3) * leftFront);
    LeftBack.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) * 0.5 - gamepad1.right_stick_x * 0.3) * rightFront);
    RightFront.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) * 0.5 + gamepad1.right_stick_x * 0.3) * leftBack);
    RightBack.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) * 0.5 + gamepad1.right_stick_x * 0.3) * rightBack);

    //multiplier for arm strength
    if(gamepad1.right_trigger>0.5) {
      multiplier = 2;
    } else {
      multiplier = 1;
    }
    //lift control
    if(gamepad1.a) {
      ClawArm.setPower(0.3*multiplier);
    }
    else if(gamepad1.b) {
      ClawArm.setPower(-0.3*multiplier);
    }
    else {
      ClawArm.setPower(0);
    }

    //controls whole arm movement up and down
    if(gamepad1.x) {
      ClawArm2.setPower(0.35 * multiplier);
      //moves the whole arm up and down
    } else if(gamepad1.y) {
      ClawArm2.setPower(-0.35 * multiplier);
    }
    else {
      ClawArm2.setPower(-0.05);
    }
    if(gamepad1.left_trigger > 0.5){
      servo.setPosition(1);
      servo2.setPosition(0);
    } else {
      servo.setPosition(0);
      servo2.setPosition(0.8);
    }
  }

  private void callibrate() {
    if(gamepad2.a) {
      leftBack += 0.001 * gamepad2.left_stick_x;
    }
    if(gamepad2.b) {
      rightBack += 0.001 * gamepad2.left_stick_x;
    }
    if(gamepad2.x) {
      rightFront += 0.001 * gamepad2.left_stick_x;
    }
    if(gamepad2.y) {
      leftFront += 0.001 * gamepad2.left_stick_x;
    }
  }

  private void print() {
    // Prints data for debug purposes
    telemetry.addData("leftFront", leftFront);
    telemetry.addData("rightFront", rightFront);
    telemetry.addData("leftBack", leftBack);
    telemetry.addData("rightBack", rightBack);
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
