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

    public DcMotor armMotor = null;
    public Servo handServo = null;

    double wheelReducter = 0.6;
    double armReducter = 1;
    double armPower = 0;

    static final double MAX_ARM_POS = 228;

    private void drive() {
        double leftWheel = -gamepad1.left_stick_y * wheelReducter;
        double rightWheel = -gamepad1.right_stick_y * wheelReducter;

        if (gamepad1.right_bumper) { // increasing max speed of robot
            wheelReducter = 0.8;
        } else if (gamepad1.left_bumper) { // decreasing max speed of robot
            wheelReducter = 0.35;
        } else {
            wheelReducter = 0.6; // resetting to default value
        }

        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);

        telemetry.addData("left: ",  "%.2f", leftWheel);
        telemetry.addData("right: ", "%.2f", rightWheel);
        telemetry.addData("wheelReducter: ", "%.2f", wheelReducter);
    }
    private void arm() {
        armPower = ((gamepad2.left_stick_y + gamepad2.right_stick_y) / 2) * armReducter;

        if (gamepad2.right_bumper) {
            armReducter = 0.8;
        } else if (gamepad2.left_bumper) {
            armReducter = 0.35;
        } else {
            armReducter = 0.6;
        }

//        if (armMotor.getCurrentPosition() >= -MAX_ARM_POS) {
//            armMotor.setPower(armPower);
//        }
//        if (armMotor.getCurrentPosition() > -10) {
//            armPower *= -1;
//            armMotor.setPower(armPower);
//        } else if (armMotor.getCurrentPosition() <= -MAX_ARM_POS) {
//            armPower *= -1;
//            armMotor.setPower(armPower);
//        }
        if ((armMotor.getCurrentPosition() > 0 && armPower > 0) || armMotor.getCurrentPosition() <= -MAX_ARM_POS) {
            armPower *= -1;
        }
//        if (armMotor.getCurrentPosition() <= -MAX_ARM_POS) {
//            armPower *= -1;
//        }

        armMotor.setPower(armPower);


        telemetry.addData("armPos: ", armMotor.getCurrentPosition());
        telemetry.addData("arm: ", "%.2f", armPower);
        telemetry.addData("armReducter: ", "%.2f", armReducter);
    }
    private void hand() {
        if (gamepad2.a) {
            handServo.setPosition(0.5);
        } else if (gamepad2.b) {
            handServo.setPosition(0);
        }
    }
    private void initDrive() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setTargetPosition(0);
        rightDrive.setTargetPosition(0);

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    private void initArm() {
        armMotor = hardwareMap.get(DcMotor.class, "arm");

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
