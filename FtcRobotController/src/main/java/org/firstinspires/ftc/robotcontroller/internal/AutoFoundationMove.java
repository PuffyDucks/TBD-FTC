/**
 * Documentation:
 *
 * * All power between -1 and 1
 * ** All durations in seconds
 *
 * Move robot
 * move(forward power, side power, turn power, duration)
 *
 * Change angle of arm
 * stdFlip(power of motor, duration)
 *
 * Change height of claw
 * stdHeight(power of motor, duration)
 *
 * Open/close claw
 * void std(boolean if claw is open, duration)
 */

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
  private DcMotor ClawArm;
  private DcMotor ClawArm2;
  private Servo servo;
  private Servo servo2;

  private float multiplier = 1;


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
    waitForStart();

    move(-1, 0, 0, 1);
    //Moves the main arm up from a resting position
    arm((float) -0.5,1);
    move(1,0,0,1);
    print();
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

  // Moves robot
  //xy plane is centered on robot with arm facing forwards
  private void move(float y, float x, float rotation, long duration) {
    LeftFront.setPower((y - x) * 0.5 - rotation * 0.3);
    LeftBack.setPower((y + x) * 0.5 - rotation * 0.3);
    RightFront.setPower((y + x) * 0.5 + rotation * 0.3);
    RightBack.setPower((y - x) * 0.5 + rotation * 0.3);
    telemetry.addData("LF", LeftFront.getPower());
    telemetry.addData("RF", RightFront.getPower());
    telemetry.addData("LB", LeftBack.getPower());
    telemetry.addData("RB", RightBack.getPower());
    telemetry.addData("Duration", duration);
    telemetry.update();
    sleep(duration*1000);
    LeftFront.setPower(0);
    LeftBack.setPower(0);
    RightBack.setPower(0);
    RightFront.setPower(0);
  }

  // Flips arm's angle
  //pos is down on flip power
  //neg is up
  private void arm(float power, long duration) {
    ClawArm2.setPower(power);
    telemetry.addData("ArmMove", ClawArm.getPower());
    telemetry.addData("duration", duration);
    sleep(duration*1000);
  }

  // Moves claw's height
  //pos is up on vert power
  private void intakeV(float power, long duration) {
    ClawArm.setPower(power);
    telemetry.addData("Intake", ClawArm2.getPower());
    telemetry.addData("Duration", duration);
    sleep(duration*1000);
  }

  // Toggles servos
  private void intake(boolean power, long delay) {
    if(power == true){
      servo.setPosition(1);
      servo2.setPosition(0);
    } else {
      servo.setPosition(0);
      servo2.setPosition(0.8);
    }
    //delay before starting
    sleep(delay * 1000);
  }

  private void print() {
    // Prints data for debug purposes
    telemetry.update();
  }
}
