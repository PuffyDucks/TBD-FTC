/*
 * Documentation:
 *
 * All power and position between -1 and 1
 * All durations in seconds
 * Continuous is boolean
 *
 * Move robot
 * move(forward power, side power, turn power, duration)
 *
 * Move arm motor
 * motor(power, duration, continuous)
 *
 * Move arm servo
 * servo(position)
 */

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "AutoDepotRed", group = "")
public class AutoDepotRed extends LinearOpMode {
  private DcMotor LeftFront;
  private DcMotor LeftBack;
  private DcMotor RightFront;
  private DcMotor RightBack;
  private ColorSensor ColorFront;
  private ColorSensor ColorBottom;
  private double magnitude;
  private double highest;

  private double wheelSetA;
  private double wheelSetB;

  private double multiplierWheels = 0.8;
  private double multiplierTurn = 0.7;

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
    ColorFront = hardwareMap.colorSensor.get("colorFront");
    ColorBottom = hardwareMap.colorSensor.get("colorBottom");


    ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
    ArmServo = hardwareMap.servo.get("ArmServo");
    //these are functions below
    call();
    set();
    servo(1);
    //motor(0.3,1.5,false);
    move(0.3, 0, 0, 3);
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
    LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
    RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
  }

  private void move(double y, double x, double rotation, double duration) {
    magnitude = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
    highest = Math.abs(y) + Math.abs(x);

    if (highest == 0) {
      highest = 1;
    }

    wheelSetA = (y - x) * (1 - x * multiplierTurn) * magnitude / highest;
    wheelSetB = (y + x) * (1 - x * multiplierTurn) * magnitude / highest;

    LeftFront.setPower((wheelSetA + rotation * multiplierTurn) * multiplierWheels);
    LeftBack.setPower((wheelSetB + rotation * multiplierTurn) * multiplierWheels);
    RightFront.setPower((wheelSetB - rotation * multiplierTurn) * multiplierWheels);
    RightBack.setPower((wheelSetA - rotation * multiplierTurn) * multiplierWheels);

    sleep((long) (duration*1000));
    LeftFront.setPower(0);
    LeftBack.setPower(0);
    RightFront.setPower(0);
    RightBack.setPower(0);
  }

  private void motor(double power, double duration, boolean continuous) {
    ArmMotor.setPower(power);
    sleep((long) (duration*1000));
    if(continuous != true) {
      ArmMotor.setPower(0);
    }
  }

  private void servo(double position) {
    ArmServo.setPosition(position);
  }
}