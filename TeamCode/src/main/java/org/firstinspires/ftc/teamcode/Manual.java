package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Manual Mode", group = "")
public class Manual extends LinearOpMode {
  private DcMotor LeftWheel;
  private DcMotor RightWheel;
  private DcMotor DefaultArm;
  private Servo LeftClaw;
  private Servo RightClaw;
  private DcMotor InceptionArm;

  double wheelSpeed;
  double armSpeed;


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
      call();
      set();
      while (opModeIsActive()) {
          setVariables();
          defaultArm();
          inceptionArm();
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
          armSpeed = 5;
      } else {
          wheelSpeed = 1;
          armSpeed = 2;
      }
  }

  private void defaultArm() {
      // Movement of default arm
      // Right bumper, left bumper, Y and X button inputs
      if (gamepad1.right_bumper && gamepad1.left_bumper) {
          DefaultArm.setPower(-0.1);
      } else if (gamepad1.left_bumper) {
          LeftClaw.setPosition(0.8);
          RightClaw.setPosition(0.8);
      } else if (gamepad1.y) {
          DefaultArm.setPower(0.25 * armSpeed);
      } else if (gamepad1.x) {
          DefaultArm.setPower(-0.25 * armSpeed);
      } else {
          LeftClaw.setPosition(0.3);
          RightClaw.setPosition(0.3);
          DefaultArm.setPower(0);
      }
  }

  private void inceptionArm() {
      // Movement of inception arm
      // B and A button inputs
      if (gamepad1.b) {
          InceptionArm.setPower(0.3);
      } else if (gamepad1.a) {
          InceptionArm.setPower(-0.3);
      } else {
          InceptionArm.setPower(0);
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
