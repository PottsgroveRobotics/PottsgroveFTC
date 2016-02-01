package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class PottsgroveTeleOp extends OpMode {
    //hello Eric
    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */
    // TETRIX VALUES.
    final static double HANDRIGHT_MIN_RANGE  = 0.20;
    final static double HANDRIGHT_MAX_RANGE  = 0.90;
    final static double HANDLEFT_MIN_RANGE  = 0.20;
    final static double HANDLEFT_MAX_RANGE  = 0.7;

    static double shoulderSpeed = 1.0;
    static double elbowSpeed = 1.0;
    static double tapeExtrusionSpeed = 1.0;
    static double tapeAngleSpeed = 0.5;


    // amount to change the arm servos' position.
    double handLeftPosition = 0.1;
    double handRightPosition = 0.9;

    //declare motors and servos
    DcMotor motorDriveRight;
    DcMotor motorDriveLeft;
    DcMotor motorArmShoulder;
    DcMotor motorArmElbow;
    Servo motorClawLeft;
    Servo motorClawRight;
    DcMotor motorTapeAngle;
    DcMotor motorTapeExtrusion;


    /**
     * Constructor
     */
    public PottsgroveTeleOp() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        motorDriveRight = hardwareMap.dcMotor.get("motor_2");
        //motorDriveRight.setDirection(DcMotor.Direction.REVERSE);
        motorDriveLeft = hardwareMap.dcMotor.get("motor_1");
        //motorDriveLeft.setDirection(DcMotor.Direction.REVERSE);
        motorTapeAngle = hardwareMap.dcMotor.get("motor_5");
        //motorTapeAngle.setDirection(DcMotor.Direction.REVERSE);
        motorTapeExtrusion = hardwareMap.dcMotor.get("motor_6");
        //motorTapeExtrusion.setDirection(DcMotor.Direction.REVERSE);
        motorArmShoulder = hardwareMap.dcMotor.get("motor_3");
        //motorArmShoulder.setDirection(DcMotor.Direction.REVERSE);
        motorArmElbow = hardwareMap.dcMotor.get("servo_4");
        //motorArmElbow.setDirection(DcMotor.Direction.REVERSE);


        motorClawLeft = hardwareMap.servo.get("servo_3");
        motorClawRight = hardwareMap.servo.get("servo_4");

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        motorDriveRight.setPower(mapJoysticktoDriveMotor(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x, -1, 1)));
        motorDriveLeft.setPower(mapJoysticktoDriveMotor(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x, -1, 1)));


        
        // write the values to the motors
        //motorDriveRight.setPower(driveRight);
        motorDriveRight.setPower(1.0);
        // motorDriveLeft.setPower(driveLeft);
        motorDriveLeft.setPower(1.0);


        motorTapeExtrusion.setPower(-gamepad1.right_stick_y * tapeExtrusionSpeed);
        motorTapeAngle.setPower(gamepad1.right_stick_x *tapeAngleSpeed);

      

        //insert Arm functionality here

        // update the position of the claw
        /**if (gamepad1.x) {
         *   clawPosition += clawDelta;
         *}
         *
         *if (gamepad1.b) {
         *    clawPosition -= clawDelta;
         *}
         */

        // clip the position values so that they never exceed their allowed range.
        handLeftPosition = Range.clip(handLeftPosition, HANDLEFT_MIN_RANGE, HANDLEFT_MAX_RANGE);
        handRightPosition = Range.clip(handRightPosition, HANDRIGHT_MIN_RANGE, HANDRIGHT_MAX_RANGE);
        //clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

        // write position values to the wrist and claw servo
        /*arm.setPosition(armPosition);

        claw.setPosition(clawPosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("right arm", "right arm:  " + String.format("%.2f", handRightPosition));
        telemetry.addData("left arm", "left arm:  " + String.format("%.2f", handLeftPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", driveLeft));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", driveRight));
        telemetry.addData("elbow tgt pwr", "elbow pwr: " + String.format("%.2f", armElbow));
        telemetry.addData("shoulder tgt pwr", "shoulder pwr: " + String.format("%.2f", armShoulder));
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */


    double mapJoysticktoDriveMotor(double joystickValue) {
        return joystickValue * Math.abs(joystickValue);
    }

    double restrictServo(double position) {
        //return new output
    }
}
