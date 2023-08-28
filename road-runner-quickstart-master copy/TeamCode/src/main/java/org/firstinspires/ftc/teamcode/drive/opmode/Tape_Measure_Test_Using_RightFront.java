package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.Math;

@TeleOp(name = "Tape_Measure_Test_Using_RightFront", group = "")
//@Disabled
public class Tape_Measure_Test_Using_RightFront extends LinearOpMode {

    private static DcMotor rightFront;

    @Override
    public void runOpMode() {
        rightFront = hardwareMap.dcMotor.get("rightFront");

        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//rightFront.setPower(1);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setTargetPosition(0);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {


                if (gamepad1.right_trigger != 0) {
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger != 0) {
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setPower(gamepad1.left_trigger * -1);
                } else  {
                    rightFront.setPower(1);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (gamepad1.dpad_up) {
                    rightFront.setTargetPosition(0);
                } else if (gamepad1.dpad_right) {
                    rightFront.setTargetPosition(250);
                } else if (gamepad1.dpad_down) {
                    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFront.setPower(1);
                    rightFront.setTargetPosition(500);
                } else if (gamepad1.dpad_left) {
                    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFront.setPower(1);
                    rightFront.setTargetPosition(750);
                }

                telemetry.addData("RightFront Power", rightFront.getPower());
                telemetry.addData("RightFront Current Position", rightFront.getCurrentPosition());
                telemetry.addData("RightFront Target Position", rightFront.getTargetPosition());


                telemetry.update();
            }
        }
    }
}