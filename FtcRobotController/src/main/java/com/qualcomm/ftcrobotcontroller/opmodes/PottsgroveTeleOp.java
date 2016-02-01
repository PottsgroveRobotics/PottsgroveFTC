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
    static double tapeDirectionSpeed = 0.5;


    // amount to change the arm servos' position.
    double handLeftPosition = 0.1;
    double handRightPosition = 0.1;

    //declare motors and servos
    DcMotor motorDriveRight;
    DcMotor motorDriveLeft;
    DcMotor motorArmShoulder;
    DcMotor motorArmElbow;
    Servo motorClawLeft;
    Servo motorClawRight;
    DcMotor motorTapeAngle;
    DcMotor motorTapeDistance;

    // position of the arm servos.
    double shoulderPos = motorArmShoulder.getPosition();
    double elbowPos = motorArmElbow.getPosition();
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
        motorTapeDistance = hardwareMap.dcMotor.get("motor_6");
        //motorTapeDistance.setDirection(DcMotor.Direction.REVERSE);

        motorArmShoulder = hardwareMap.dcMotor.get("servo_1");
        motorArmElbow = hardwareMap.servo.get("servo_2");
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

		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        float driveThrottle = gamepad1.left_stick_y;
        float driveDirection = gamepad1.left_stick_x;
        float driveRight = driveThrottle - driveDirection;
        float driveLeft = driveThrottle + driveDirection;
        float armShoulderIncrement = gamepad2.left_stick_y/2;
        float armElbowIncrement = gamepad1.right_stick_y/2;

        // clip the right/left values so that the values never exceed +/- 1
        driveRight = Range.clip(driveRight, -1, 1);
        driveLeft = Range.clip(driveLeft, -1, 1);


        // write the values to the motors
        //motorDriveRight.setPower(driveRight);
        motorDriveRight.setPower(1.0);
        // motorDriveLeft.setPower(driveLeft);
        motorDriveLeft.setPower(1.0);

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
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    float restrictServo(float position){
        //return new output;
    }
}
