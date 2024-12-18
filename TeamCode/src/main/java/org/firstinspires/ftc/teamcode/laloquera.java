package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "laloquera (Blocks to Java)")
public class laloquera extends LinearOpMode {

  private DcMotor RF;
  private DcMotor RB;
  private DcMotor LF;
  private DcMotor LB;
  private DcMotor Left;
  private DcMotor Right;
  private Servo garra;
  private DcMotor Brazo;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    float Brazo2;
    float Speed;
    float Turn;
    float Strafe;

    RF = hardwareMap.get(DcMotor.class, "RF");
    RB = hardwareMap.get(DcMotor.class, "RB");
    LF = hardwareMap.get(DcMotor.class, "LF");
    LB = hardwareMap.get(DcMotor.class, "LB");
    Left = hardwareMap.get(DcMotor.class, "Left");
    Right = hardwareMap.get(DcMotor.class, "Right");
    garra = hardwareMap.get(Servo.class, "garra");
    Brazo = hardwareMap.get(DcMotor.class, "Brazo");

    // Put initialization blocks here.
    RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Habilita Encoders para chasis
    RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // Brake Freno para chasis
    LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Encoder Slider
    Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // Invertir izquierda
    Left.setDirection(DcMotor.Direction.REVERSE);
    LF.setDirection(DcMotor.Direction.REVERSE);
    LB.setDirection(DcMotor.Direction.REVERSE);
    // Resetea  encoder Slider
    Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Garra en reversa por  alguna razón
    garra.setDirection(Servo.Direction.REVERSE);
    // Resetear encoder brazo y brake freno
    Brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Freno de límite Garra
    garra.scaleRange(0.15, 0.8);
    // Posición inicial de garra
    garra.setPosition(0.77);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Establecer variables
        Brazo2 = -gamepad2.left_stick_y;
        Speed = -gamepad1.left_stick_y;
        Turn = gamepad1.right_stick_x;
        Strafe = gamepad1.left_stick_x;
        // Movimiento de chasis, normal y lento
        if (gamepad1.right_bumper) {
          RF.setPower(((Speed - Turn) - Strafe) * 0.5);
          RB.setPower(((Speed - Turn) + Strafe) * 0.5);
          LF.setPower((Speed + Turn + Strafe) * 0.5);
          LB.setPower(((Speed + Turn) - Strafe) * 0.5);
        } else {
          RF.setPower((Speed - Turn) - Strafe);
          RB.setPower((Speed - Turn) + Strafe);
          LF.setPower(Speed + Turn + Strafe);
          LB.setPower((Speed + Turn) - Strafe);
        }
        Brazo.setPower(Brazo2 * 0.8);
        // Bajar velocidad del brazo
        if (gamepad2.dpad_down) {
          Brazo.setPower(Brazo2 * 0.4);
        }
        // Subir slider hasta arriba
        if (gamepad2.a) {
          Left.setTargetPosition(17000);
          Right.setTargetPosition(17000);
          Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          ((DcMotorEx) Left).setVelocity(40000);
          ((DcMotorEx) Right).setVelocity(40000);
        }
        // Subir Sllider para posicionar High Chamber
        if (gamepad2.y) {
          Left.setTargetPosition(6600);
          Right.setTargetPosition(6600);
          Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          ((DcMotorEx) Left).setVelocity(3500);
          ((DcMotorEx) Right).setVelocity(3500);
        }
        // Subir Sllider para colocar High Chamber
        if (gamepad2.x) {
          Left.setTargetPosition(8200);
          Right.setTargetPosition(8200);
          Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          ((DcMotorEx) Left).setVelocity(3500);
          ((DcMotorEx) Right).setVelocity(3500);
        }
        // Bajar Slider a 0
        if (gamepad2.b) {
          Left.setTargetPosition(0);
          Right.setTargetPosition(0);
          Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          ((DcMotorEx) Left).setVelocity(5000);
          ((DcMotorEx) Right).setVelocity(5000);
        }
        // Cerrar garra
        if (gamepad2.right_bumper) {
          garra.setPosition(0.2);
        }
        // Abrir  garra
        if (gamepad2.left_bumper) {
          garra.setPosition(0.77);
        }
        // Posición de encoder frontal
        telemetry.addData("LF", LF.getCurrentPosition());
        telemetry.addData("LB", LB.getCurrentPosition());
        telemetry.addData("RF", RF.getCurrentPosition());
        telemetry.addData("RB", RB.getCurrentPosition());
        // Posición de encoder garra
        telemetry.addData("Garra", garra.getPosition());
        // Posición de encoder brazo
        telemetry.addData("Left", Left.getCurrentPosition());
        telemetry.addData("Right", Right.getCurrentPosition());
        telemetry.addData("Brazo", Brazo.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}
