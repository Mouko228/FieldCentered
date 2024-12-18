package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="field OpMode", group="Iterative OpMode")

public class ITerative extends OpMode {
    // Declare OpMode members.
    double initYaw;
    double angles;
    double adjustedYaw;
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightFront;
    private DcMotor RightBack;

    private IMU imu;
    private IMU.Parameters MyParameters;
    private ElapsedTime runtime = new ElapsedTime();

    private boolean toggle = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //elpepe
        telemetry.addData("Status", "Initializing");

        imu = hardwareMap.get(IMU.class,"imu");

        MyParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );

        imu.initialize(MyParameters);

        angles = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        initYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        LeftFront = hardwareMap.get(DcMotor.class, "LF");
        LeftBack = hardwareMap.get(DcMotor.class,"LB");
        RightFront = hardwareMap.get(DcMotor.class,"RF");
        RightBack = hardwareMap.get(DcMotor.class,"RB");

        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();

    }


    @Override
    public void loop() {
        double speed = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;


        //elpepe
        angles = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        adjustedYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-initYaw;

        // toggle field/normal

        double zerodYaw = -initYaw+imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double theta = Math.atan2(speed, strafe) * 180/Math.PI; // aka angle

        double realTheta;

        realTheta = (360 - zerodYaw) + theta;

        double power = Math.hypot(strafe, speed);

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double LF_power = (power * cos / maxSinCos + turn)*0.5;
        double RF_power = (power * sin / maxSinCos - turn)*0.5;
        double LB_power = (power * sin / maxSinCos + turn)*0.5;
        double RB_power = (power * cos / maxSinCos - turn)*0.5;


        if ((power + Math.abs(turn)) > 1) {
            LF_power /= power + turn;
            RF_power /= power - turn;
            LB_power /= power + turn;
            RB_power /= power - turn;
        }

        LeftFront.setPower(LF_power);
        LeftBack.setPower(LB_power);
        RightFront.setPower(RF_power);
        RightBack.setPower(RB_power);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
