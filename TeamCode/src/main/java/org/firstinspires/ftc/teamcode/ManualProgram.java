package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Manual Program", group = "")
  public class ManualProgram extends LinearOpMode {
  private DcMotor LeftWheel;
  private DcMotor RightWheel;
  private DcMotor DefaultArm;
  private DcMotor InceptionArm;
  private Servo LeftClaw;
  private Servo RightClaw;

  double wheelSpeed;
  double armSpeed;
  double armSpeed2;
  boolean clawsClosed;

  /**
  * This function is executed when this Op Mode is selected from the Driver Station.
  */

  @Override
  public void runOpMode() {
    LeftWheel = hardwareMap.dcMotor.get("Left Wheel");
    RightWheel = hardwareMap.dcMotor.get("Right Wheel");
    DefaultArm = hardwareMap.dcMotor.get("Default Arm");
    LeftClaw = hardwareMap.servo.get("Left Claw");
    RightClaw = hardwareMap.servo.get("Right Claw");
    InceptionArm = hardwareMap.dcMotor.get("Inception Arm");
    clawsClosed = false;

    //these are functions below
    call();
    set();

    while (opModeIsActive()) {
      //see below
      setVariables();
      defaultArm();
      inceptionArm();
      claws();
      move();
      print();
    }
  }

  private void call() {
    // Call program, and devices
    waitForStart();
    ((DcMotorEx) DefaultArm).setMotorEnable();
    ((DcMotorEx) LeftWheel).setMotorEnable();
    ((DcMotorEx) RightWheel).setMotorEnable();
  }

  private void set() {
    // Set direction of devices
    DefaultArm.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftClaw.setDirection(Servo.Direction.REVERSE);
    RightClaw.setDirection(Servo.Direction.FORWARD);
    InceptionArm.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
    RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  private void setVariables() {
    // Sets turbo mode with variables
    // Right bumper input
    if (gamepad1.right_bumper) {
      wheelSpeed = 5;
      armSpeed = 3;
    } else if (gamepad1.left_bumper) {
      wheelSpeed = 0.4;
      armSpeed = 0.75;
    } else {
      wheelSpeed = 1;
      armSpeed = 2;
    }
    if (gamepad2.right_bumper) {
      armSpeed2 = 2;
    } else if (gamepad2.left_bumper) {
      armSpeed2 = 0.5;
    } else {
      armSpeed2 = 1.25;
    }
  }

  private void defaultArm() {
    // Movement of default arm
    // Y and X button inputs
    if (gamepad1.y) {
      DefaultArm.setPower(0.33 * armSpeed);
    } else if (gamepad1.x) {
      DefaultArm.setPower(-0.33 * armSpeed);
    } else {
      DefaultArm.setPower(0);
    }
  }

  private void inceptionArm() {
    // Movement of inception arm
    // Left analog stick y input
    InceptionArm.setPower((gamepad2.left_stick_y * 0.66 - 0.2) * 0.45 * armSpeed2);
  }

  private void claws() {
    if (gamepad2.a || gamepad2.x) {
      // Open claws
      LeftClaw.setPosition(1);
      RightClaw.setPosition(1);
    } else if (gamepad2.b || gamepad2.y) {
      // Close claws
      LeftClaw.setPosition(0.3);
      RightClaw.setPosition(0.3);
    }
  }

  private void move() {
    // Movement of robot with wheels
    // Left analog stick input
    LeftWheel.setPower(gamepad1.left_stick_y * 0.5 * wheelSpeed - gamepad1.left_stick_x * 0.5 * wheelSpeed);
    RightWheel.setPower(gamepad1.left_stick_y * 0.5 * wheelSpeed + gamepad1.left_stick_x * 0.5 * wheelSpeed);
  }

  private void print() {
    // Prints data for debug purposes
    telemetry.addData("joystick", gamepad1.left_stick_x);
    telemetry.addData("Left Wheel Power", LeftWheel.getPower());
    telemetry.addData("Right Wheel Power", RightWheel.getPower());
    telemetry.addData("Left Wheel Position", LeftWheel.getCurrentPosition());
    telemetry.addData("Right Wheel Position", RightWheel.getCurrentPosition());
    telemetry.addData("Arm Position", DefaultArm.getCurrentPosition());
    telemetry.update();
  }
}
