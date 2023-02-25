/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Robot: Autonomous BY XCELLENCE (Parking)", group="Robot")
public class xCellenceAutonomous extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private Servo hand_servo = null;
    private ElapsedTime runtime = new ElapsedTime();
    // konus -> 91-90 cm / 1.5 tile || robot -> 39 cm   || medium junction -> 116 cm / 2 tile || 1 tile = 60.9 = 56-57 cm
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double FEET_PER_METER = 3.28084;
    static final int MAX_ARM_POSITION = 1750;// 1250 -> 138cm 1750 -> 59
    static final int ARM_POS_OFFSET = 100;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    static final double COUNTS_PER_MOTOR_REV = 22;
    static final double METER_TO_INCHES = 39.3701;
    static final double DRIVE_GEAR_REDUCTION = 12;
    static final double WHEEL_DIAMETER_INCHES = 3.54;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double ROBOT_SIDE = 0.37;
    static final double ONE_TILE = 0.611;

    int Left=16;
    int Middle=18;
    int Right=17;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);}

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        armMotor = hardwareMap.get(DcMotor.class, "arm");
        hand_servo = hardwareMap.get(Servo.class, "hand_servo");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hand_servo.setPosition(0.5);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) { // searching for qr code
                    if(tag.id == Left || tag.id == Right || tag.id == Middle) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break; // leaving from loop after finding
                    }
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            } // logging data

            telemetry.update();
        }

        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it 1was never sighted during the init loop :(");
            telemetry.update();
        }

        while (opModeIsActive()) {
            if (tagOfInterest != null) {
                if (tagOfInterest.id == Left) {
                    // encoderDrive(FORWARD_SPEED, 0.4, 0.4, 5);
                    // arm();
                    //encoderDrive(FORWARD_SPEED, 1, 1, 2);
                    armMotor.setTargetPosition(250);
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(1);
                    sleep(1000);

                    armMotor.setPower(0);
                    encoderDrive(FORWARD_SPEED, -0.65, -0.65, 5);
                    sleep(1000);
                    encoderDrive(TURN_SPEED, 0.42, -0.42, 5);
                    sleep(1000);
                    encoderDrive(FORWARD_SPEED, 0.27,0.27,5);

                    //encoderDrive(FORWARD_SPEED, (ONE_TILE/15), (ONE_TILE/15), 5);
                    //armMiddleJ();

                    break;
                } else if (tagOfInterest.id == Middle) {

                    encoderDrive(FORWARD_SPEED, -0.65, -0.65, 5);

                    //sleep(2000);
                    //encoderDrive(TURN_SPEED, 0.37, -0.37, 5);
                    //sleep(2000);
                    //encoderDrive(FORWARD_SPEED, 0.1,0.1,5);
                    //encoderDrive(FORWARD_SPEED, 1.5, 1.5, 5);
                    break;
                } else if (tagOfInterest.id == Right) {
                    encoderDrive(FORWARD_SPEED, -0.65, -0.65, 5);
                    sleep(1000);
                    encoderDrive(TURN_SPEED, -0.48, 0.48, 5);
                    sleep(1000);
                    encoderDrive(FORWARD_SPEED, 0.35,0.35,5);

                    //encoderDrive(FORWARD_SPEED, 1.8, 1.8, 5);
                    //encoderDrive(TURN_SPEED, 1, -1, 5);
                    //encoderDrive(FORWARD_SPEED, 0.4, 0.4, 5);


                    break;
                }

            }



            telemetry.addData("leftPos: ", leftDrive.getCurrentPosition());
            telemetry.addData("leftPosT: ", leftDrive.getTargetPosition());

            telemetry.addData("rightPos: ", rightDrive.getCurrentPosition());
            telemetry.addData("rightPosT: ", rightDrive.getTargetPosition());
        }
    }

    private void armMiddleJ() {
        if (opModeIsActive()) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setTargetPosition(MAX_ARM_POSITION);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armMotor.setPower(FORWARD_SPEED);

        while (opModeIsActive() && armMotor.getCurrentPosition() <= (MAX_ARM_POSITION - ARM_POS_OFFSET)) {
            telemetry.addData("ArmPos: ", armMotor.getCurrentPosition());
            telemetry.update();
        }
            armMotor.setPower(0);

            //sleep(1000);
            //        hand_servo.setPosition(0.5); // 0.5
            //        sleep(1000);
            hand_servo.setPosition(0);
            //sleep(2000);

            armMotor.setPower(FORWARD_SPEED);
            armMotor.setTargetPosition(0);

        while (opModeIsActive() && armMotor.getCurrentPosition() >= ARM_POS_OFFSET) {
            telemetry.addData("ArmPos: ", armMotor.getCurrentPosition());
            telemetry.update();
        }

            armMotor.setPower(0);
            //sleep(2000);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void encoderDrive(double speed,
                             double leftMeters, double rightMeters,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftMeters * COUNTS_PER_INCH * METER_TO_INCHES);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightMeters * COUNTS_PER_INCH * METER_TO_INCHES);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}