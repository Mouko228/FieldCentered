package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="elepepe OpMode", group="Iterative OpMode")

public class ITerative extends OpMode
{
    // Declare OpMode members.
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightFront;
    private DcMotor RightBack;

    private IMU imu;
    private IMU.Parameters MyParameters;
    private ElapsedTime runtime = new ElapsedTime();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        imu = hardwareMap.get(IMU.class,"imu");

        MyParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );

        imu.initialize(MyParameters);

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
    public void start() { runtime.reset();}


    @Override
    public void loop() {

        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;

        double speed = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;

        LF_power = Range.clip(speed + strafe + turn, -1,1);
        LB_power = Range.clip(speed - strafe + turn, -1,1);
        RF_power = Range.clip(speed - strafe - turn, -1,1);
        RB_power = Range.clip(speed + strafe - turn, -1,1);

        // Send calculated power to wheels
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
