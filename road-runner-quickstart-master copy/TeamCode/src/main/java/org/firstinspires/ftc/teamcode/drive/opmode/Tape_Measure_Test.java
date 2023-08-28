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

@TeleOp(name = "Tape_Measure_Test", group = "")
//@Disabled
public class Tape_Measure_Test extends LinearOpMode {

    private static DcMotor TapeMeasure;

    @Override
    public void runOpMode() {
        TapeMeasure = hardwareMap.dcMotor.get("TapeMeasure");

        TapeMeasure.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        TapeMeasure.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TapeMeasure.setTargetPosition(0);
        TapeMeasure.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TapeMeasure.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                //TapeMeasure.setPower(gamepad1.right_trigger);
                if (gamepad1.right_trigger != 0) {
                    TapeMeasure.setPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger != 0) {
                    TapeMeasure.setPower(gamepad1.left_trigger * -1);
                }
                telemetry.addData("TapeMeasure", TapeMeasure.getPower());
                telemetry.update();
            }
        }
    }
}