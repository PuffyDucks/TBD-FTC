/**
 * Documentation:
 *
 * * All power between -1 and 1
 * ** All durations in seconds
 *
 * Move robot
 * move(forward power, side power, turn power, duration)
 *
 * everything else got destroyed cuz robot is being rebuilt rn lol
*/

/*
 * Note: reading color sensor is with
 * color_sensor.red();   // Red channel value
 * color_sensor.green(); // Green channel value
 * color_sensor.blue();  // Blue channel value

 * color_sensor.alpha(); // Total luminosity
 * color_sensor.argb();  // Combined color value
*/

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "AutoFoundationMove", group = "")
public class AutoFoundationMove extends LinearOpMode {
  private DcMotor LeftFront;
  private DcMotor LeftBack;
  private DcMotor RightFront;
  private DcMotor RightBack;
  private ColorSensor Color;
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

    //these are functions below
    call();
    set();
    waitForStart();
    // Moves forwards to foundation
    move(1, 0, 0, 5);
    /* add stuff here that hooks foundation or something idk how it'll work */
    // Move foundation backwards into zone
    move((float) -0.3, 0, 0, 4);

    // Turns left
    move(0, 0, -1, 0.3);

    // Moves forwards until line is hit

    print();
  }

  private void call() {
    // Call program, and devices
    //waitForStart();
    ((DcMotorEx) LeftFront).setMotorEnable();
    ((DcMotorEx) LeftBack).setMotorEnable();
    ((DcMotorEx) RightFront).setMotorEnable();
    ((DcMotorEx) RightBack).setMotorEnable();
  }

  private void set() {
    // Set direction of devices
    LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
    RightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    Color = hardwareMap.colorSensor.get("color");
  }

  // Moves robot
  //xy plane is centered on robot with arm facing forwards
  private void move(float y, float x, float rotation, long duration) {
    checkOp();
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

  private void print() {
    // Prints data for debug purposes
    telemetry.update();
  }
  private void checkOp(){
    if(!opModeIsActive()){
      LeftFront.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      RightFront.setPower(0);
      stop();
    }
  }
}
