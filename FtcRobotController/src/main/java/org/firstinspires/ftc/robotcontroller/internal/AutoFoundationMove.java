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

// From VUFORIA
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
// End VUFORIA

@Autonomous(name = "AutoFoundationMove", group = "")
  public class AutoFoundationMove extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightFront;
    private DcMotor RightBack;

    private DcMotor ArmMotor;
    private Servo ArmServo;

    // More VUFORIA
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";

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
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    // End VUFORIA


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
    vuforiaInit();

    targetsSkyStone.activate();

    while (opModeIsActive()) {
      //see below
      move(1, 0, 0, 1);

      // Vuforia again
      // check all the trackable targets to see which one (if any) is visible.
      targetVisible = false;
      for (VuforiaTrackable trackable : allTrackables) {
          if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
              telemetry.addData("Visible Target", trackable.getName());
              targetVisible = true;

              // getUpdatedRobotLocation() will return null if no new information is available since
              // the last time that call was made, or if the trackable is not currently visible.
              OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
              if (robotLocationTransform != null) {
                  lastLocation = robotLocationTransform;
              }
              break;
          }
      }

      // Provide feedback as to where the robot is located (if we know).
      if (targetVisible) {
          // express position (translation) of robot in inches.
          VectorF translation = lastLocation.getTranslation();
          telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                  translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

          // express the rotation of the robot in degrees.
          Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
          telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
      }
      else {
          telemetry.addData("Visible Target", "none");
      }
      telemetry.update();
      //End Vuforia

      print();
    }
    // Disable Tracking when we are done;
    targetsSkyStone.deactivate();
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

  private void vuforiaInit() {
    // Prep VUFORIA
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
     */
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
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

    // We need to rotate the camera around it's long axis to bring the correct camera forward.
    if (CAMERA_CHOICE == BACK) {
        phoneYRotate = -90;
    } else {
        phoneYRotate = 90;
    }

    // Rotate the phone vertical about the X axis if it's in portrait mode
    if (PHONE_IS_PORTRAIT) {
        phoneXRotate = 90 ;
    }

    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
    final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

    /**  Let all the trackable listeners know where the phone is.  */
    for (VuforiaTrackable trackable : allTrackables) {
        ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }
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
    telemetry.update();
  }
}
