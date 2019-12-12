/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.*;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@Autonomous(name="NavStartRedLeft", group ="Skystone")
//@Disabled
public class SkyStoneNavigationLeftRed extends LinearOpMode {
  // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
  // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
  // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
  //
  // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
  //
  private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
  private static final boolean PHONE_IS_PORTRAIT = false;

  /*
   * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
   * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
   * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
   * web site at https://developer.vuforia.com/license-manager.
   *
   * Vuforia license keys are always 380 characters long, and look as if they contain mostly
   * random data. As an example, here is a example of a fragment of a valid key:
   *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
   * Once you've obtained a license key, copy the string from the Vuforia web site
   * and paste it in to your code on the next line, between the double quotes.
   */
  private static final String VUFORIA_KEY =
          "AY4Pdeb/////AAABmeuDt5dlTEiMkbqvIwLznng+AUAT9IX6CRJMh4NUrPUHrGnnqKN/GPlBFovkf7g8cmW7QilNwBnEFw7lTgjqPLxcKgdK7hTxqzD7pMka50VQLDLMlZQxnlO8jcBmg2BtBuw4vGf6jnJNGjlKqMCtY3shK/uhQfRDhDJoFf81czzHo/S81R8B7lyvva5hDoQ0pUglGZ1x+t6s0+ezf4Au9cyBq2t/mAvb9iz7cL4LPl4xbIfspls8UO2Do3DaWoWg428dVDr9O3Ju2qLtsBp4AekVy43XTzjikmj43XwyY3ZoTo+6cC9i2g/xglkBdr/FljrWTsB8r1YPzXHXJEgSDEhAHjD8FMbJVGOplJm8ZEwx";

  // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
  // We will define some constants and conversions here
  private static final float mmPerInch        = 25.4f;
  private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

  // Constant for Stone Target
  private static final float stoneZ = 2.00f * mmPerInch;

  // Constants for the center support targets
  private static final float bridgeZ = 6.42f * mmPerInch;
  private static final float bridgeY = 23 * mmPerInch;
  private static final float bridgeX = 5.18f * mmPerInch;
  private static final float bridgeRotY = 59;                                 // Units are degrees
  private static final float bridgeRotZ = 180;

  // Constants for perimeter targets
  private static final float halfField = 72 * mmPerInch;
  private static final float quadField  = 36 * mmPerInch;

  // Class Members
  private DcMotor LeftFront;
  private DcMotor LeftBack;
  private DcMotor RightFront;
  private DcMotor RightBack;
  private double leftFront = 1;
  private double rightFront = 1;
  private double leftBack = 1;
  private double rightBack = 1;
  private DcMotor ClawArm;
  private DcMotor ClawArm2;
  private Servo servo;
  private Servo servo2;
  private OpenGLMatrix lastLocation = null;
  private VuforiaLocalizer vuforia = null;
  private boolean targetVisible = false;
  private float tx;
  private float ty;
  private float tz;
  private float rx;
  private float ry;
  private float rz;
  private float dx;
  private float dy;
  private float dz;
  private float phoneXRotate    = 0;
  private float phoneYRotate    = 0;
  private float phoneZRotate    = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
     * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
     */
    LeftFront = hardwareMap.dcMotor.get("LeftFront");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightFront = hardwareMap.dcMotor.get("RightFront");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    ClawArm = hardwareMap.dcMotor.get("ClawArm");
    ClawArm2 = hardwareMap.dcMotor.get("ClawArm2");
    servo = hardwareMap.get(Servo.class, "Grab");
    servo2 = hardwareMap.get(Servo.class, "Grab2");
    waitForStart();
    ((DcMotorEx) LeftFront).setMotorEnable();
    ((DcMotorEx) LeftBack).setMotorEnable();
    ((DcMotorEx) RightFront).setMotorEnable();
    ((DcMotorEx) RightBack).setMotorEnable();
    LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
    RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters.vuforiaLicenseKey = "AY4Pdeb/////AAABmeuDt5dlTEiMkbqvIwLznng+AUAT9IX6CRJMh4NUrPUHrGnnqKN/GPlBFovkf7g8cmW7QilNwBnEFw7lTgjqPLxcKgdK7hTxqzD7pMka50VQLDLMlZQxnlO8jcBmg2BtBuw4vGf6jnJNGjlKqMCtY3shK/uhQfRDhDJoFf81czzHo/S81R8B7lyvva5hDoQ0pUglGZ1x+t6s0+ezf4Au9cyBq2t/mAvb9iz7cL4LPl4xbIfspls8UO2Do3DaWoWg428dVDr9O3Ju2qLtsBp4AekVy43XTzjikmj43XwyY3ZoTo+6cC9i2g/xglkBdr/FljrWTsB8r1YPzXHXJEgSDEhAHjD8FMbJVGOplJm8ZEwx";
    parameters.cameraDirection   = CAMERA_CHOICE;

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Load the data sets for the trackable objects. These particular data
    // sets are stored in the 'assets' part of our application.
    VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

    VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
    stoneTarget.setName("Stone Target");
    VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
    blueRearBridge.setName("Blue Rear Bridge");
    VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
    redRearBridge.setName("Red Rear Bridge");
    VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
    redFrontBridge.setName("Red Front Bridge");
    VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
    blueFrontBridge.setName("Blue Front Bridge");
    VuforiaTrackable red1 = targetsSkyStone.get(5);
    red1.setName("Red Perimeter 1");
    VuforiaTrackable red2 = targetsSkyStone.get(6);
    red2.setName("Red Perimeter 2");
    VuforiaTrackable front1 = targetsSkyStone.get(7);
    front1.setName("Front Perimeter 1");
    VuforiaTrackable front2 = targetsSkyStone.get(8);
    front2.setName("Front Perimeter 2");
    VuforiaTrackable blue1 = targetsSkyStone.get(9);
    blue1.setName("Blue Perimeter 1");
    VuforiaTrackable blue2 = targetsSkyStone.get(10);
    blue2.setName("Blue Perimeter 2");
    VuforiaTrackable rear1 = targetsSkyStone.get(11);
    rear1.setName("Rear Perimeter 1");
    VuforiaTrackable rear2 = targetsSkyStone.get(12);
    rear2.setName("Rear Perimeter 2");

    // For convenience, gather together all the trackable objects in one easily-iterable collection */
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    allTrackables.addAll(targetsSkyStone);

        /*
          In order for localization to work, we need to tell the system where each target is on the field, and
          where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
          Transformation matrices are a central, important concept in the math here involved in localization.
          See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
          for detailed information. Commonly, you'll encounter transformation matrices as instances
          of the {@link OpenGLMatrix} class.

          If you are standing in the Red Alliance Station looking towards the center of the field,
              - The X axis runs from your left to the right. (positive from the center to the right)
              - The Y axis runs from the Red Alliance Station towards the other side of the field
                where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
              - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)

          Before being transformed, each target image is conceptually located at the origin of the field's
           coordinate system (the center of the field), facing up.
         */

    // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
    // Rotated it to to face forward, and raised it to sit on the ground correctly.
    // This can be used for generic target-centric approach algorithms
    stoneTarget.setLocation(OpenGLMatrix
            .translation(0, 0, stoneZ)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

    //Set the position of the bridge support targets with relation to origin (center of field)
    blueFrontBridge.setLocation(OpenGLMatrix
            .translation(-bridgeX, bridgeY, bridgeZ)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

    blueRearBridge.setLocation(OpenGLMatrix
            .translation(-bridgeX, bridgeY, bridgeZ)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

    redFrontBridge.setLocation(OpenGLMatrix
            .translation(-bridgeX, -bridgeY, bridgeZ)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

    redRearBridge.setLocation(OpenGLMatrix
            .translation(bridgeX, -bridgeY, bridgeZ)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

    //Set the position of the perimeter targets with relation to origin (center of field)
    red1.setLocation(OpenGLMatrix
            .translation(quadField, -halfField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

    red2.setLocation(OpenGLMatrix
            .translation(-quadField, -halfField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

    front1.setLocation(OpenGLMatrix
            .translation(-halfField, -quadField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

    front2.setLocation(OpenGLMatrix
            .translation(-halfField, quadField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

    blue1.setLocation(OpenGLMatrix
            .translation(-quadField, halfField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

    blue2.setLocation(OpenGLMatrix
            .translation(quadField, halfField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

    rear1.setLocation(OpenGLMatrix
            .translation(halfField, quadField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

    rear2.setLocation(OpenGLMatrix
            .translation(halfField, -quadField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

    //
    // Create a transformation matrix describing where the phone is on the robot.
    //
    // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
    // Lock it into Portrait for these numbers to work.
    //
    // Info:  The coordinate frame for the robot looks the same as the field.
    // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
    // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
    //
    // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
    // pointing to the LEFT side of the Robot.
    // The two examples below assume that the camera is facing forward out the front of the robot.

    // We need to rotate the camera around it's long axis to bring the correct camera forward.
    if (CAMERA_CHOICE == BACK) {
      phoneYRotate = -90;
    } else {
      phoneYRotate = 90;
    }

    // Rotate the phone vertical about the X axis if it's in portrait mode
    if (PHONE_IS_PORTRAIT) {
      phoneXRotate = 90;
    }
    phoneXRotate = 180;

    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
    final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 2.0f * mmPerInch;     // eg: Camera is ON the robot's center line

    OpenGLMatrix robotFromCamera = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

    //Let all the trackable listeners know where the phone is.
    for (VuforiaTrackable trackable : allTrackables) {
      ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }

    // WARNING:
    // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
    // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
    // CONSEQUENTLY do not put any driving commands in this loop.
    // To restore the normal opmode structure, just un-comment the following line:
    targetsSkyStone.activate();
    //waitForStart();
    move((float) -1, 0, 0, (long) 2.3);
    while (!vuf(targetVisible, allTrackables) && opModeIsActive()) {
    }
    //finetune(targetVisible, allTrackables, lastLocation);
    intake(false,0);
    //line
    arm(-0.5f,1);
    //move forward
    servo.setPosition(0);
    servo.setPosition(1);
    move( -0.5f,0,0, (long) 2);
    //servo's out
    intake(false,1);
    move(0.5f,0,0, (long) 0.5);
    intake(true,1);
    arm(0.2f, (long) 0.5);
    sleep(100);
    //backwards
    move(0.5f,0,0,4);
    move(0,0,1, (long) 4);
    move(-1,0,0,5);

    // Note: To use the remote camera preview:
    // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
    // Tap the preview window to receive a fresh image.



    // Disable Tracking when we are done;
    targetsSkyStone.deactivate();
  }
  private boolean vuf(boolean targetVisible, List<VuforiaTrackable> allTrackables) {
    checkOp();
    // check all the trackable targets to see which one (if any) is visible.
    targetVisible = false;
    for (VuforiaTrackable trackable : allTrackables) {
      if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
        tx = trackable.getLocation().get(0,3)/mmPerInch;
        ty = trackable.getLocation().get(1,3)/mmPerInch;
        tz = trackable.getLocation().get(2, 3) /mmPerInch;
        telemetry.addData("Visible Target", trackable.getName());
        telemetry.addData("Target Pos", "{X, Y, Z} = %.1f, %.1f, %.1f",
                tx, ty, tz);
        targetVisible = true;

        // getUpdatedRobotLocation() will return null if no new information is available since
        // the last time that call was made, or if the trackable is not currently visible.
        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
        if (robotLocationTransform != null) {
          OpenGLMatrix lastLocation = robotLocationTransform;
        }
        break;
      }
    }
    // Provide feedback as to where the robot is located (if we know).
    if (targetVisible) {
      checkOp();
      stopMotor();
      // express position (translation) of robot in inches.
      telemetry.addData("line419",null);
      telemetry.update();
      VectorF translation = lastLocation.getTranslation();
      rx = translation.get(0)/mmPerInch;
      ry = translation.get(1)/mmPerInch;
      rz = translation.get(2)/mmPerInch;
      dx = tx-rx;
      dy = ty-tz;
      dz = tz-rz;
      telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
              rx, ry, rz);
      telemetry.addData("Delta Pos", "{X, Y, Z} = %.1f, %.1f, %.1f",
              dx,dy, dz);

      // express the rotation of the robot in degrees.
      Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
      telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
      telemetry.addData("rx",rx);
      telemetry.update();
      return true;
    }
    else {
      checkOp();
      moveAlt(0, (float) -0.6,-0.05f, (long) 0.1);
      return false;
    }
  }


  private void finetune(boolean targetVisible, List<VuforiaTrackable> allTrackables){
    double min=-11.5;
    double max=-10.5;
    while(opModeIsActive() && max<rx || min>rx) {
      for (VuforiaTrackable trackable : allTrackables) {
        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
          tx = trackable.getLocation().get(0,3)/mmPerInch;
          ty = trackable.getLocation().get(1,3)/mmPerInch;
          tz = trackable.getLocation().get(2, 3) /mmPerInch;
          telemetry.addData("Visible Target", trackable.getName());
          telemetry.addData("Target Pos", "{X, Y, Z} = %.1f, %.1f, %.1f",
                  tx, ty, tz);
          targetVisible = true;

          // getUpdatedRobotLocation() will return null if no new information is available since
          // the last time that call was made, or if the trackable is not currently visible.
          OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
          if (robotLocationTransform != null) {
            OpenGLMatrix lastLocation = robotLocationTransform;
          }
          break;
        }
      }
      if (targetVisible) {
        // express position (translation) of robot in inches.
        VectorF translation = lastLocation.getTranslation();
        rx = translation.get(0) / mmPerInch;
        ry = translation.get(1) / mmPerInch;
        rz = translation.get(2) / mmPerInch;
        dx = tx - rx;
        dy = ty - tz;
        dz = tz - rz;
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                rx, ry, rz);
        telemetry.addData("Delta Pos", "{X, Y, Z} = %.1f, %.1f, %.1f",
                dx, dy, dz);

        // express the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        telemetry.addData("rx",rx);
      }
      if (rx < max || rx > min && opModeIsActive()) {
        if (rx > max) {
          moveAlt(0, -0.5f, 0, (long) 0.1);
        } else if (rx < min) {
          moveAlt(0, 0.5f, 0, (long) 0.1);
        }
      }
      else{
        stopMotor();
      }
      telemetry.update();
    }
  }



  //x&y mirror joystick
  private void move(float y, float x, float rotation, long duration) {
    telemetry.addData("LeftFront",((y - x) * 0.5 - rotation * 0.3) * leftFront);
    telemetry.addData("LeftBack", ((y + x) * 0.5 - rotation * 0.3) * rightFront);
    telemetry.addData("RightFront",((y + x) * 0.5 + rotation * 0.3) * rightFront);
    telemetry.addData("RightBack", ((y - x) * 0.5 + rotation * 0.3) * rightBack);
//    LeftFront.setPower(((y - x) * 0.5 - rotation * 0.3) * leftFront);
//    LeftBack.setPower(((y + x) * 0.5 - rotation * 0.3) * leftBack);
//    RightFront.setPower(((y + x) * 0.5 + rotation * 0.3) * RightFront);
//    RightBack.setPower(((y - x) * 0.5 + rotation * 0.3) * RightBack);
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
    checkOp();
  }
  private void moveAlt(float y, float x, float rotation, long duration) {
    LeftFront.setPower(((y - x) * 0.5 - rotation * 0.3) * leftFront);
    LeftBack.setPower(((y + x) * 0.5 - rotation * 0.3) * rightFront);
    RightFront.setPower(((y + x) * 0.5 + rotation * 0.3) * leftBack);
    RightBack.setPower(((y - x) * 0.5 + rotation * 0.3) * rightBack);
//    telemetry.addData("LF", LeftFront.getPower());
//    telemetry.addData("RF", RightFront.getPower());
//    telemetry.addData("LB", LeftBack.getPower());
//    telemetry.addData("RB", RightBack.getPower());
//    telemetry.addData("Duration", duration);
//    telemetry.update();
    sleep(duration*1000);
//    LeftFront.setPower(0);
//    LeftBack.setPower(0);
//    RightBack.setPower(0);
//    RightFront.setPower(0);
    checkOp();
  }
  private void stopMotor(){
    LeftFront.setPower(0);
    LeftBack.setPower(0);
    RightBack.setPower(0);
    RightFront.setPower(0);
    checkOp();
  }
  // Flips arm's angle
  //pos is up on flip power
  private void arm(float power, long duration) {
    checkOp();
    ClawArm2.setPower(power);
    telemetry.addData("ArmMove", ClawArm.getPower());
    telemetry.addData("duration", duration);
    sleep(duration*1000);
  }

  // Moves claw's height
  //pos is up on vert power
  private void intakeV(float power, long duration) {
    checkOp();
    ClawArm.setPower(power);
    telemetry.addData("Intake", ClawArm2.getPower());
    telemetry.addData("Duration", duration);
    sleep(duration*1000);
  }

  // Toggles servos
  private void intake(boolean power, long delay) {
    checkOp();
    if(power == true){
      servo.setPosition(0.5);
      servo2.setPosition(0.3);
    } else {
      servo.setPosition(0.1);
      servo2.setPosition(0.8);
    }
    //delay before starting
    sleep(delay * 1000);
  }
  private void checkOp(){
    if(!opModeIsActive()){
      LeftFront.setPower(0);
      LeftBack.setPower(0);
      RightBack.setPower(0);
      RightFront.setPower(0);
      ClawArm.setPower(0);
      ClawArm2.setPower(0);
      stop();
    }
  }
}
