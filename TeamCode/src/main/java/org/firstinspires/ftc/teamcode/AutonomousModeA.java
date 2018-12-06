package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous Mode A", group = "")
public class AutonomousModeA extends LinearOpMode {
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
    //LANDER LAND HERE
    /**
    //Idea scrapped for December competition, possibly try for January
    DefaultArm.setPower(-1.5);
    telemetry.addData("Debug", 0);
    telemetry.update();
    waitForStart();
    **/
    // Move 5.66 feet
    LeftWheel.setPower(0.5);
    RightWheel.setPower(0.5);
    sleep(3394);
    // Deploy team marker
    InceptionArm.setPower(0.5);
    sleep(500);
    LeftClaw.setPosition(0.8);
    RightClaw.setPosition(0.8);
    sleep(500);
    LeftClaw.setPosition(0.3);
    RightClaw.setPosition(0.3);
    InceptionArm.setPower(0.5);
    sleep(500);
    // 45 degree right turn
    LeftWheel.setPower(0.5);
    RightWheel.setPower(0.25);
    sleep(500);
    // Move 7 feet
    LeftWheel.setPower(0.5);
    RightWheel.setPower(0.5);
    sleep(4200);
    // Boost into depot
    LeftWheel.setPower(1);
    RightWheel.setPower(1);
    sleep(200);
    /**
    // Knock a goldio boy
    // Orientation AGAIN
      //Do Things With Color Sensor (IMPORT CODE FOR COLOR SENSOR)
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
    */
  }
}
