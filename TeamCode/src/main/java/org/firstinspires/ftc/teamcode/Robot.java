package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "VZHUH VZHUH")

public class Robot extends OpMode {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    public DcMotor leftArm = null;
    public DcMotor rightArm = null;
    public Servo handServo = null;

    double wheelReducter = 0.8;
    double armReducter = 1;

    private void drive() {
        double leftWheel = -gamepad1.left_stick_y * wheelReducter;
        double rightWheel = -gamepad1.right_stick_y * wheelReducter;

        if (gamepad1.right_bumper) {
            wheelReducter = 1;
        } else {
            wheelReducter = 0.8;
        }

        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);

        telemetry.addData("left",  "%.2f", leftWheel);
        telemetry.addData("right", "%.2f", rightWheel);
        telemetry.addData("wheelReducter", "%.2f", wheelReducter);
    }

    private void arm() {
        double armPower = ((gamepad2.left_stick_y + gamepad2.right_stick_y) / 2) * armReducter;

        leftArm.setPower(armPower);
        rightArm.setPower(armPower);

        telemetry.addData("Arm: ", "%.2f", armPower);
    }

    private void hand() {
        boolean hand = gamepad2.a;

        if (gamepad2.a) {
            handServo.setPosition(0.5);
        }
        else if (gamepad2.b) {
            handServo.setPosition(0);
        }

        telemetry.addData("hand", hand);
    }

    private void initDrive() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initArm() {
        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");

        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initHand() {
        handServo = hardwareMap.get(Servo.class, "hand_servo");

        handServo.setPosition(0);
    }


    @Override
    public void init() {
        initDrive();
        initArm();
        initHand();
        telemetry.addData(">", "(\n\nMASHINKA DAIYN. BAS KNOPKANY.");
    }

    @Override
    public void loop() {
        drive();
        arm();
        hand();
    }
}
