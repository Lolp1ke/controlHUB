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

@Autonomous(name = "Only Parking")
public class Parking extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private Servo hand_servo = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double FEET_PER_METER = 3.28084;

    static final double fx = 578.272;
    static final double fy = 578.272;
    static final double cx = 402.145;
    static final double cy = 221.506;
    static final double tagSize = 0.166;

    static final int Left = 16;
    static final int Right = 18;
    static final int Middle = 17;

    static final int sleepMS = 2000;

    static final double COUNTS_PER_MOTOR_REV = 22;
    static final double METER_TO_INCHES = 39.3701;
    static final double DRIVE_GEAR_REDUCTION = 12;
    static final double WHEEL_DIAMETER_INCHES = 3.54;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);}

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
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
        hand_servo.setPosition(0.8);

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
                double forwardMove = 1.1;
                double turn = 0.5;
                double timeOut = 3.0;

                if (tagOfInterest.id == Left) {
                    encoderDrive(FORWARD_SPEED, -forwardMove, -forwardMove, timeOut);
                    encoderDrive(TURN_SPEED, -turn, turn, timeOut);
                    encoderDrive(FORWARD_SPEED, -forwardMove, -forwardMove, timeOut);

                    break;
                } else if (tagOfInterest.id == Middle) {

                    break;
                } else if (tagOfInterest.id == Right) {

                    break;
                }
            }
        }
    }
    private void armUp() {
        int middleJ = 1500;
        int lowJ = 1000;
        int timeoutS = 3;

        if (opModeIsActive()) {
            armMotor.setTargetPosition(-middleJ);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(Math.abs(0.6));

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftDrive.isBusy() && rightDrive.isBusy())) {
                telemetry.addData("Currently at",  " at %7d", armMotor.getCurrentPosition());
                telemetry.update();
            }

            armMotor.setPower(0);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(sleepMS);
        }
    }
    private void encoderDrive(double speed, double leftMeters, double rightMeters, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftMeters * COUNTS_PER_INCH * METER_TO_INCHES); // converting inches to meters
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightMeters * COUNTS_PER_INCH * METER_TO_INCHES);

            leftDrive.setTargetPosition(newLeftTarget); // setting target place
            rightDrive.setTargetPosition(newRightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION); // launching motors
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftDrive.isBusy() && rightDrive.isBusy())) {
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(sleepMS);
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
