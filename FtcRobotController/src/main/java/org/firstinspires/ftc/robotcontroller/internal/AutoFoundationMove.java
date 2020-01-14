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

    while (opModeIsActive()) {
      //see below
      move(1, 0, 0, 1);

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

  private void move(float y, float x, float rotation, float duration) {
    LeftFront.setPower(((y - x) * 0.5 - rotation * 0.3) * leftFront);
    LeftBack.setPower(((y + x) * 0.5 - rotation * 0.3) * rightFront);
    RightFront.setPower(((y + x) * 0.5 + rotation * 0.3) * leftBack);
    RightBack.setPower(((y - x) * 0.5 + rotation * 0.3) * rightBack);
    sleep(duration*1000);
  }

  private void motor(float armMovement, float duration) {
    ArmMotor.setPower(armMovement);
    sleep(duration*1000);
  }

  private void servo(float servoPostive) {
    ArmServo.setPosition(servoPostive);
  }

  private void print() {
    // Prints data for debug purposes
    telemetry.addData("gamepad1 left stick x", gamepad1.left_stick_x);
    telemetry.update();
  }
}
