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
    static double robotYawSpeed = 0.2;
    static double driveSpeed = 1.0;
    static double speedDelta = 0.02;
    static double handDelta = 0.0;
    static double handDeltaSpeed = 0.01;


    // amount to change the arm servos' position.
    double handLeftPosition = 0.1;
    double handRightPosition = 0.9;

    //declare motors and servos
    DcMotor motorDriveRight;
    DcMotor motorDriveLeft;
    DcMotor motorArmShoulder;
    DcMotor motorArmElbow;
    Servo servoHandLeft;
    Servo servoHandRight;
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


        servoHandLeft = hardwareMap.servo.get("servo_3");
        servoHandRight = hardwareMap.servo.get("servo_4");

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        //DRIVE

        double motorRightPower = mapJoysticktoDriveMotor(Range.clip(driveSpeed*(-gamepad1.left_stick_y - gamepad1.left_stick_x), -1, 1));
        double motorLeftPower =  mapJoysticktoDriveMotor(Range.clip(driveSpeed*(-gamepad1.left_stick_y + gamepad1.left_stick_x), -1, 1));

        motorLeftPower = (1/(motorLeftPower+gamepad2.left_stick_x*robotYawSpeed))*(gamepad2.left_stick_x * robotYawSpeed - (motorLeftPower+gamepad2.left_stick_x*robotYawSpeed))+1;
        motorRightPower = (1/(motorRightPower-gamepad2.left_stick_x*robotYawSpeed))*(-gamepad2.left_stick_x * robotYawSpeed - (motorRightPower-gamepad2.left_stick_x*robotYawSpeed))+1;

        motorDriveRight.setPower(motorRightPower);
        motorDriveLeft.setPower(motorLeftPower);

        //ARM

        double motorShoulderPower = -gamepad2.left_stick_y * shoulderSpeed;
        double motorElbowPower = -gamepad2.right_stick_y * elbowSpeed;

        handDelta = gamepad2.right_stick_x*handDeltaSpeed;

        handLeftPosition -= handDelta;
        handRightPosition += handDelta;


        motorArmShoulder.setPower(motorShoulderPower);
        motorArmElbow.setPower(motorElbowPower);

        //TAPE

        motorTapeExtrusion.setPower(-gamepad1.right_stick_y * tapeExtrusionSpeed);
        motorTapeAngle.setPower(gamepad1.right_stick_x *tapeAngleSpeed);

        motorDriveRight.setPower(motorRightPower);
        motorDriveLeft.setPower(motorLeftPower);

        //ARROWS

        //Drive

        if(gamepad1.dpad_up){
            driveSpeed += speedDelta;
            Range.clip(driveSpeed, -1, 1);
        } else if(gamepad1.dpad_down){
            driveSpeed -= speedDelta;
            Range.clip(driveSpeed, -1, 1);
        }

        //Shoulder

        if(gamepad2.dpad_up){
            shoulderSpeed += speedDelta;
            Range.clip(shoulderSpeed, -1, 1);
        }else if(gamepad2.dpad_down){
            shoulderSpeed -= speedDelta;
            Range.clip(shoulderSpeed, -1, 1);
        }

        //Elbow

        if(gamepad2.dpad_right){
            elbowSpeed += speedDelta;
            Range.clip(elbowSpeed, -1, 1);
        } else if(gamepad2.dpad_left){
            elbowSpeed -= speedDelta;
            Range.clip(elbowSpeed, -1, 1);
        }

        // yaw speed
        
        if(gamepad2.left_bumper){
            robotYawSpeed -= speedDelta;
        }else if(gamepad2.right_bumper){
            robotYawSpeed += speedDelta;
        }


        // clip the position values so that they never exceed their allowed range.
        handLeftPosition = Range.clip(handLeftPosition, HANDLEFT_MIN_RANGE, HANDLEFT_MAX_RANGE);
        handRightPosition = Range.clip(handRightPosition, HANDRIGHT_MIN_RANGE, HANDRIGHT_MAX_RANGE);

        robotYawSpeed = Range.clip(robotYawSpeed, 0,1);

        servoHandLeft.setPosition(handLeftPosition);
        servoHandRight.setPosition(handRightPosition);




		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");

        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", motorLeftPower));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", motorRightPower));

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


}
