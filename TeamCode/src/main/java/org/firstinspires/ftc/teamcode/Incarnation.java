package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Incarnation", group = "")
  public class Incarnation extends LinearOpMode {
  private DcMotor LeftWheel;
  private DcMotor RightWheel;
  private DcMotor LanderArm;
  private DcMotor Succ;

  double wheelSpeed;
  double armSpeed;
  double armSpeed2;

  /**
  * This function is executed when this Op Mode is selected from the Driver Station.
  */

  @Override
  public void runOpMode() {
    LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
    RightWheel = hardwareMap.dcMotor.get("RightWheel");
    LanderArm = hardwareMap.dcMotor.get("LanderArm");
    Succ = hardwareMap.dcMotor.get("Succ");

    //these are functions below
    call();
    set();

    while (opModeIsActive()) {
      //see below
      setVariables();
      landerArm();
      succ();
      move();
      print();
    }
  }

  private void call() {
    // Call program, and devices
    waitForStart();
    ((DcMotorEx) LanderArm).setMotorEnable();
    ((DcMotorEx) LeftWheel).setMotorEnable();
    ((DcMotorEx) RightWheel).setMotorEnable();
  }

  private void set() {
    // Set direction of devices
    LanderArm.setDirection(DcMotorSimple.Direction.FORWARD);
    Succ.setDirection(DcMotorSimple.Direction.FORWARD);
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

  private void landerArm() {
    // Movement of lander arm
    // Y and X button inputs
    if (gamepad1.y) {
      LanderArm.setPower(0.33 * armSpeed);
    } else if (gamepad1.x) {
      LanderArm.setPower(-0.33 * armSpeed);
    } else {
      LanderArm.setPower(0);
    }
  }

  private void succ() {
    // Movement of succ arm
    // Left analog stick y input
    Succ.setPower(gamepad2.left_stick_y * armSpeed2);
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
    telemetry.update();
  }
}
