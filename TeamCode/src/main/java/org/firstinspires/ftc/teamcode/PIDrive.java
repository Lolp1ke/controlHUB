package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PICONTROL")
public class PIDrive extends OpMode {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    private ElapsedTime timer = new ElapsedTime();

    double relativeLeft;
    double relativeRight;
    double errorL;
    double errorR;
    double integralL = 0;
    double integralR = 0;
    double currentPosL;
    double currentPosR;
    static final double meterPerDegree = 3.1415 * 0.09 / 360;

    static final
    double wheelReducter = 0.6;

    double Ki = 0.0005;
    double Kp = 0.03;
    double maxTime = 3;

    private void PIControl() {
        double leftWheel = -gamepad1.left_stick_y;
        double rightWheel = -gamepad1.right_stick_y;

        errorL -= (leftDrive.getCurrentPosition() - relativeLeft) * meterPerDegree;
        errorR -= (rightDrive.getCurrentPosition() - relativeRight) * meterPerDegree;

        integralL += errorL;
        integralR += errorR;

        telemetry.addData("errorL: ", errorL);
        telemetry.addData("errorR: ", errorR);

        telemetry.addData("powerL: ", (errorL * Kp + integralL * Ki));
        telemetry.addData("powerR: ", (errorR * Kp + integralR * Ki));

        telemetry.addData("leftWheel: ", leftWheel);
        telemetry.addData("rightWheel: ", rightWheel);
        telemetry.addData("timer: ", timer.seconds());

        if (leftWheel != 0 || rightWheel != 0) {
            leftDrive.setPower((leftWheel + (errorL * Kp + integralL * Ki)) * wheelReducter);
            rightDrive.setPower((rightWheel + (errorR * Kp + integralR * Ki)) * wheelReducter);
        } else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
        if (timer.seconds() > maxTime) {
            timer.reset();

            integralL = 0;
            integralR = 0;

            errorL = 0;
            errorR = 0;
        }
    }

    private void initDrive() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        relativeLeft = leftDrive.getCurrentPosition();
        relativeRight = rightDrive.getCurrentPosition();

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void init() {
        initDrive();
    }
    @Override
    public void loop() {
        PIControl();
    }
}
