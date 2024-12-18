package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="FieldCentredMecanum", group="Iterative OpMode")

public class TestShuji extends OpMode {
    // Declare OpMode Members
    //Variables
    double initYaw;
    double angles;
    double adjustedYaw;

    //Chassis Motors
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightFront;
    private DcMotor RightBack;

    //Mechanism Motors
    private DcMotor Left;
    private DcMotor Right;
    private DcMotor Brazo;
    private Servo Garra;

    //IMU
    private IMU imu;
    public IMU.Parameters MyParameters;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        //Mapping Components
        imu = hardwareMap.get(IMU.class,"imu");

        LeftFront = hardwareMap.get(DcMotor.class, "LF");
        LeftBack = hardwareMap.get(DcMotor.class,"LB");
        RightFront = hardwareMap.get(DcMotor.class,"RF");
        RightBack = hardwareMap.get(DcMotor.class,"RB");

        Left = hardwareMap.get(DcMotor.class,"Left");
        Right = hardwareMap.get(DcMotor.class,"Right");
        Brazo = hardwareMap.get(DcMotor.class, "Brazo");
        Garra = hardwareMap.get(Servo.class,"garra");

        //Initializing Gyro
        MyParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(MyParameters);

        angles = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        initYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        //Setting Motor's direction
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        Left.setDirection(DcMotorSimple.Direction.REVERSE);
        Garra.setDirection(Servo.Direction.REVERSE);

        //Setting Motor's IdleMode
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Resetting encoders
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting Motor's mode
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Cerrar Garra
        Garra.setPosition(0.77);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {
        //Joystick
        double speed = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;

        double brazo = gamepad2.left_stick_y;

        //Updating yaw
        angles = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        adjustedYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-initYaw;

        // toggle field/normal
        double zerodYaw = -initYaw+imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double theta = Math.atan2(speed, strafe) * 180/Math.PI; // aka angle

        double realTheta = (360 - zerodYaw) + theta;

        double power = Math.hypot(strafe, speed);

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double LF_power = Range.clip((power * cos / maxSinCos + turn),-1,1);
        double RF_power = Range.clip((power * sin / maxSinCos - turn),-1,1);
        double LB_power = Range.clip((power * sin / maxSinCos + turn),-1,1);
        double RB_power = Range.clip((power * cos / maxSinCos - turn),-1,1);

        //PowerUp Chassis Motors
        if (gamepad1.right_bumper) {
            LeftFront.setPower(LF_power*0.5);
            LeftBack.setPower(LB_power*0.5);
            RightFront.setPower(RF_power*0.5);
            RightBack.setPower(RB_power*0.5);
        } else {
            LeftFront.setPower(LF_power);
            LeftBack.setPower(LB_power);
            RightFront.setPower(RF_power);
            RightBack.setPower(RB_power);
        }

        //PowerUp MechMotors
        Brazo.setPower(brazo * 0.4);

        //Slider Up and Down
        if (gamepad2.a) {
            Left.setTargetPosition(17000);
            Right.setTargetPosition(17000);

            Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx) Left).setVelocity(40000);
            ((DcMotorEx) Right).setVelocity(40000);
        } else if (gamepad2.b) {
            Left.setTargetPosition(0);
            Right.setTargetPosition(0);

            Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx) Left).setVelocity(5000);
            ((DcMotorEx) Right).setVelocity(5000);
        }

        //Setup for Chamber, Release with Up
        if (gamepad2.dpad_right){
            Left.setTargetPosition(6600);
            Right.setTargetPosition(6600);

            Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx) Left).setVelocity(40000);
            ((DcMotorEx) Right).setVelocity(40000);
        } else if (gamepad2.dpad_up){
            Left.setTargetPosition(8200);
            Right.setTargetPosition(8200);

            Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx) Left).setVelocity(40000);
            ((DcMotorEx) Right).setVelocity(40000);
        }

        //Close/Open Garra
        if (gamepad2.right_bumper) {
            Garra.setPosition(0.2);
        }else if (gamepad2.left_bumper) {
            Garra.setPosition(0.77);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addData("left",Left.getCurrentPosition());
        telemetry.addData("right", Right.getCurrentPosition());
    }

    @Override
    public void stop() {}
}