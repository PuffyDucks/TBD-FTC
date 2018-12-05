package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous", group = "")
public class AutonomousDepot extends LinearOpMode {
  private DcMotor LeftWheel;
  private DcMotor RightWheel;
  private DcMotor DefaultArm;
  private Servo LeftClaw;
  private Servo RightClaw;
  private DcMotor InceptionArm;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    DefaultArm = hardwareMap.dcMotor.get("Default Arm");
    InceptionArm = hardwareMap.dcMotor.get("Inception Arm");
    LeftWheel = hardwareMap.dcMotor.get("Left Wheel");
    RightWheel = hardwareMap.dcMotor.get("Right Wheel");
    LeftClaw = hardwareMap.servo.get("Left Claw");
    RightClaw = hardwareMap.servo.get("Right Claw");

    ((DcMotorEx) DefaultArm).setMotorEnable();
    ((DcMotorEx) InceptionArm).setMotorEnable();
    ((DcMotorEx) LeftWheel).setMotorEnable();
    ((DcMotorEx) RightWheel).setMotorEnable();
    DefaultArm.setDirection(DcMotorSimple.Direction.FORWARD);
    InceptionArm.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftClaw.setDirection(Servo.Direction.REVERSE);
    RightClaw.setDirection(Servo.Direction.FORWARD);
    // Put block to right here
    waitForStart();
    // FRICKIN DO GO DOWN ARM MOTOR LATCHE TO LANDER
    // Put block to the right here
    sleep(1000);
    DefaultArm.setPower(0);
    // Orientation
    LeftWheel.setPower(0);
    RightWheel.setPower(0);
    sleep(500);
    LeftWheel.setPower(0);
    RightWheel.setPower(0);
    // Knock a goldio boy
    // Orientation AGAIN
    LeftWheel.setPower(0);
    RightWheel.setPower(0);
    sleep(500);
    LeftWheel.setPower(0);
    RightWheel.setPower(0);
    // Deploy THE NOT KILLROY
    LeftClaw.setPosition(0);
    RightClaw.setPosition(0);
    sleep(0);
    InceptionArm.setPower(0);
    sleep(0);
    // drive into the crater
    LeftWheel.setPower(0);
    RightWheel.setPower(0);
    sleep(2000);
    LeftWheel.setPower(0);
    RightWheel.setPower(0);
  }
}
