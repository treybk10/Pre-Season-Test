package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.Math;

@TeleOp(name = "Official_Main_Drive_CC_R3_Arm_And_Claw", group = "")
//@Disabled
public class OfficalMainDriveR3ArmAndClaw extends LinearOpMode {
    //private ElapsedTime runtime = new ElapsedTime ();
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor LeftRear;
    private DcMotor RightRear;
    private DcMotor Arm;
    private Servo LeftServo;
    private Servo RightServo;

  /*private DcMotor RightMotor;
  private DcMotor LeftMotor;*/

    private BNO055IMU imu;
    private boolean temp;
    private int count;
    boolean FC = true;
    double SpeedReducer = 0;


    //declare motor speed variables
    double RF, LF, RR, LR;

    //declare joystick position variables
    double X1, Y1, X2, Y2;

    //operational constants
    double joyScale = 0.7;  //0.5;

    double motorMax = 0.7;  //0.6;
    double Left_Stick_Angle, Left_Stick_Ratio, Left_Stick_Magnitude;
    double Left_Stick_Y, Left_Stick_X;
    double Robot_Angle, Output_Angle;
    double LTrigger = 0;
    int Count = 0;
    boolean ispressed;

    @Override
    public void runOpMode () {

        RightFront=hardwareMap.dcMotor.get("RightFront");
        LeftFront=hardwareMap.dcMotor.get("LeftFront");
        RightRear=hardwareMap.dcMotor.get("RightRear");
        LeftRear=hardwareMap.dcMotor.get("LeftRear");
        Arm=hardwareMap.get(DcMotor.class, "Arm");
        LeftServo=hardwareMap.get(Servo.class, "LeftServo");
        RightServo=hardwareMap.get(Servo.class, "RightServo");

        RightServo.setDirection(Servo.Direction.REVERSE);

        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //RearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
  /*RightMotor = hardwareMap.dcMotor.get("RightMotor");
  LeftMotor = hardwareMap.dcMotor.get("LeftMotor");*/

        temp = true;
        count = 0;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);

        //RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        if (opModeIsActive()) {
            Arm.setPower(1);
            while (opModeIsActive()) {

                if(gamepad1.right_stick_button)
                {
                    //FC = false;
                }
                if(gamepad1.left_stick_button)
                {
                    FC = true;
                }


                if(FC == true)
                {
                    LF = 0; RF = 0; LR = 0; RR = 0; X1 = 0; Y1 = 0;

                    //Setting up Variables
    /*if(gamepad1.a )
{
   motorMax = 0.3;  //0.5;
}
else
{*/
                    motorMax = 1;
//}
                    Left_Stick_Y = -gamepad1.left_stick_y;
                    Left_Stick_X = gamepad1.left_stick_x;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    Robot_Angle = angles.firstAngle * -1;
                    if(Left_Stick_Y != 0 || Left_Stick_X != 0)
                    {
                        Left_Stick_Ratio = Left_Stick_X / Left_Stick_Y;



                        //if left stick y greater than 0
                        if(Left_Stick_Y > 0){
      /*it creates this ratio left stick x/ left stick y, then it calulates the angle
      this is the same thing for the false just add 180 to the angle*/
                            Left_Stick_Angle = Math.toDegrees(Math.atan(Left_Stick_Ratio));
                        }
                        else{
                            Left_Stick_Angle = Math.toDegrees(Math.atan(Left_Stick_Ratio)) + 180;
                            if(Left_Stick_Angle > 180){Left_Stick_Angle -= 360;}
                        }
                        //it calculates the power in which direction based on the x and y
                        Left_Stick_Magnitude = Math.sqrt(Math.pow(Left_Stick_Y,2)
                                + Math.pow(Left_Stick_X,2));

                        //output angle is the way the robot wil go based on the joystick angle - the current robot angle
                        //the lines after it are just implementing them
                        Output_Angle = Left_Stick_Angle - Robot_Angle;
                        if(Output_Angle > 180){Output_Angle -= 360;}
                        if(Output_Angle < -180){Output_Angle += 360;}
                        LTrigger = (1 - gamepad1.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);

                        //this will set a value for the x and y axis of the motor
                        Y1 = Math.cos(Math.toRadians(Output_Angle)) * Left_Stick_Magnitude;
                        X1 = Math.sin(Math.toRadians(Output_Angle)) * Left_Stick_Magnitude;

                    }
                    X2 = gamepad1.right_stick_x * joyScale;

                    // Forward/back movement
                    LF += Y1; RF += Y1; LR += Y1; RR += Y1;

                    //Side to side movement
                    LF += X1; RF -= X1; LR -= X1; RR += X1;

                    //Rotation Movement
                    LF += X2; RF -= X2; LR += X2; RR -= X2;

                    //Motor Speed

                    //Clip motor power values to +/- motorMax
                    LF = Math.max(-motorMax, Math.min(LF, motorMax));
                    RF = Math.max(-motorMax, Math.min(RF, motorMax));
                    LR = Math.max(-motorMax, Math.min(LR, motorMax));
                    RR = Math.max(-motorMax, Math.min(RR, motorMax));


                    //Send values to the motors
                    if(gamepad1.left_trigger > gamepad2.left_trigger)
                    {
                        LTrigger = (0.75 - gamepad1.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);
                    }
                    else
                    {
                        LTrigger = (0.75 - gamepad2.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);
                    }

                    LeftFront.setPower(LF * LTrigger);
                    RightFront.setPower(RF * LTrigger);
                    LeftRear.setPower(LR * LTrigger);
                    RightRear.setPower(RR * LTrigger);


                }




                if(FC == false)
                {
                    LF = 0; RF = 0; LR = 0; RR = 0;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();

                    X2 = gamepad1.right_stick_x * joyScale;


                    if(gamepad1.left_trigger > gamepad1.left_trigger)
                    {
                        LTrigger = (0.75 - gamepad1.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);
                    }
                    else
                    {
                        LTrigger = (0.75 - gamepad2.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);
                    }
                    //Get joystick values
                    Y1 = -gamepad1.left_stick_y * joyScale; //invert so up is positive
                    X1 = gamepad1.left_stick_x * joyScale;
                    Y2 = -gamepad1.right_stick_y * joyScale;  //Y2 is not used at present


                    // Forward/back movement
                    LF += Y1; RF += Y1; LR += Y1; RR += Y1;

                    //Side to side movement
                    LF += X1; RF -= X1; LR -= X1; RR += X1;

                    //Rotation Movement
                    LF += X2; RF -= X2; LR += X2; RR -= X2;

                    motorMax = 1;

                    //Clip motor power values to +/- motorMax
                    LF = Math.max(-motorMax, Math.min(LF, motorMax));
                    RF = Math.max(-motorMax, Math.min(RF, motorMax));
                    LR = Math.max(-motorMax, Math.min(LR, motorMax));
                    RR = Math.max(-motorMax, Math.min(RR, motorMax));


                    //Send values to the motors
                    LeftFront.setPower(LF * LTrigger);
                    RightFront.setPower(RF * LTrigger);
                    LeftFront.setPower(LR * LTrigger);
                    RightFront.setPower(RR * LTrigger);





                }


                if (gamepad1.right_bumper) {
                    RightServo.setPosition(0.2);
                }
                if(gamepad1.left_bumper){

                    LeftServo.setPosition(0.15);
                }
                if(gamepad1.x) {
                    LeftServo.setPosition(0);
                    RightServo.setPosition(0);
                }

                if(gamepad1.a){
                    Arm.setTargetPosition(10);
                    LeftServo.setPosition(0);
                    RightServo.setPosition(0);

                }

                if(gamepad1.y){
                    Arm.setTargetPosition(750);
                }

                if (Arm.getCurrentPosition() < 100) {
                    Arm.setPower(0.3);
                }

                telemetry.addData("Right Servo", RightServo.getPosition());
                telemetry.addData("Left Servo", LeftServo.getPosition());
                telemetry.addData("armpower", Arm.getPower());
                telemetry.addData("arm Position", Arm.getCurrentPosition());
                telemetry.addData("Robot Angle",Robot_Angle);
                telemetry.update();


            }
        }
    }
}


