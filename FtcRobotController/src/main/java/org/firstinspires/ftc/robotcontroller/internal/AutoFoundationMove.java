/**
 * Documentation:
 *
 * * All power between -1 and 1
 * ** All durations in seconds
 *
 * Move robot
 * move(forward power, side power, turn power, duration)
 *
 * Move arm motor
 * motor(power, duration)
 *
 * Move arm servo
 * servo(position)
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "AutoFoundationMove", group = "")
public class AutoFoundationMove extends LinearOpMode {
  private DcMotor LeftFront;
  private DcMotor LeftBack;
  private DcMotor RightFront;
  private DcMotor RightBack;

  private DcMotor ArmMotor;
  private Servo ArmServo;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */

  @Override
  public void runOpMode() {
    LeftFront = hardwareMap.dcMotor.get("LeftFront");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightFront = hardwareMap.dcMotor.get("RightFront");
    RightBack = hardwareMap.dcMotor.get("RightBack");

    ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
    ArmServo = hardwareMap.servo.get("ArmServo");

    //these are functions below
    call();
    set();

    move(-1, 0, 0, 1);
    motor(-0.5,2);
    move(1,0,0,0);
    //move(0,1,0,3);
    move(0,0, 0.5f/*MAKE THIS TURN RIGHT BUT LEFT ON THE OTHER SIDE*/,3);
    print();
  }

  private void call() {
    // Call program, and devices
    waitForStart();
    ((DcMotorEx) LeftFront).setMotorEnable();
    ((DcMotorEx) LeftBack).setMotorEnable();
    ((DcMotorEx) RightFront).setMotorEnable();
    ((DcMotorEx) RightBack).setMotorEnable();
    ((DcMotorEx) ArmMotor).setMotorEnable();
  }

  private void set() {
    // Set direction of devices
    LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
    RightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
  }

  private void move(float y, float x, float rotation, float duration) {
    LeftFront.setPower(((y - x) * 0.5 - rotation * 0.3));
    LeftBack.setPower(((y + x) * 0.5 - rotation * 0.3));
    RightFront.setPower(((y + x) * 0.5 + rotation * 0.3));
    RightBack.setPower(((y - x) * 0.5 + rotation * 0.3));
    sleep((long) (duration*1000));
    LeftFront.setPower(0);
    LeftBack.setPower(0);
    RightFront.setPower(0);
    RightBack.setPower(0);
  }

  private void motor(double armMovement, double duration) {
    ArmMotor.setPower((float) armMovement);
    sleep((long) (duration*1000));
    ArmMotor.setPower(0);
  }

  private void servo(double servoPostive) {
    ArmServo.setPosition((float) servoPostive);
  }

  private void print() {
    // Prints data for debug purposes
    telemetry.addData("gamepad1 left stick x", gamepad1.left_stick_x);
    telemetry.update();
  }
}