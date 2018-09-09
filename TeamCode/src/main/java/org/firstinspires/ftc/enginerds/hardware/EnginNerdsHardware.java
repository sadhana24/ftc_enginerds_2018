package org.firstinspires.ftc.enginerds.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.enginerds.utilities.EngiNerdsConstants;

//------------------------------------------------------------------------------
// EngiNerdsHardware
//------------------------------------------------------------------------------

/**
 * Provides a single hardware access point between custom op-modes and the
 * OpMode class for the EngiNerds.
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 */
public class EnginNerdsHardware {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor.ZeroPowerBehavior initialZPBehavior = DcMotor.ZeroPowerBehavior.UNKNOWN;

    /* Private Variables */
    private boolean v_warning_generated = false;
    private String v_warning_message;

    // DC Motors
    public DcMotor v_motor_left_drive;
    public DcMotor v_motor_right_drive;
    public DcMotor v_motor_lift_drive;
    public DcMotor v_motor_arm_drive;

    // Servos
    private Servo v_glyph_servo_right;
    private Servo v_glyph_servo_left;
    private Servo v_relic_servo_top;
    private Servo v_relic_servo_bottom;
    private Servo v_jewel_servo;
    private Servo v_jewel_servo_top;

    //--------------------------------------------------------------------------
    // Initialize Hardware
    //--------------------------------------------------------------------------
    public void init(HardwareMap hardwareMap) {

        v_warning_generated = false;
        v_warning_message = "Can't map; ";

        //--------------------------------------------------------------------------
        // Right Drive Motor
        //--------------------------------------------------------------------------
        try {
            v_motor_right_drive = hardwareMap.dcMotor.get(EngiNerdsConstants.RIGHT_DRIVE);
            //v_motor_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Should prevent sliding down automatically.
            initialZPBehavior = v_motor_right_drive.getZeroPowerBehavior();
        } catch (Exception p_exeception) {
            SetWarningMessage(EngiNerdsConstants.RIGHT_DRIVE);
            RobotLog.i(p_exeception.getLocalizedMessage());

            v_motor_right_drive = null;
        }

        //--------------------------------------------------------------------------
        // Left Drive Motor
        //--------------------------------------------------------------------------
        try {
            v_motor_left_drive = hardwareMap.dcMotor.get(EngiNerdsConstants.LEFT_DRIVE);
            //v_motor_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Should prevent sliding down automatically.
            v_motor_left_drive.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception p_exeception) {
            SetWarningMessage(EngiNerdsConstants.LEFT_DRIVE);
            RobotLog.i(p_exeception.getLocalizedMessage());

            v_motor_left_drive = null;
        }

        //--------------------------------------------------------------------------
        // Left Lift Motor.
        //--------------------------------------------------------------------------
        try {
            v_motor_lift_drive = hardwareMap.dcMotor.get(EngiNerdsConstants.LIFT_DRIVE);
            //v_motor_lift_drive.setDirection(DcMotor.Direction.REVERSE);
            v_motor_lift_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Should prevent sliding down automatically.

        } catch (Exception p_exeception) {
            SetWarningMessage(EngiNerdsConstants.LIFT_DRIVE);
            RobotLog.i(p_exeception.getLocalizedMessage());

            v_motor_lift_drive = null;
        }

        //--------------------------------------------------------------------------
        // Arm Drive Motor
        //--------------------------------------------------------------------------
        try {
            v_motor_arm_drive = hardwareMap.dcMotor.get(EngiNerdsConstants.ARM_DRIVE);
            v_motor_arm_drive.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception p_exeception) {
            SetWarningMessage(EngiNerdsConstants.ARM_DRIVE);
            RobotLog.i(p_exeception.getLocalizedMessage());

            v_motor_arm_drive = null;
        }

        //--------------------------------------------------------------------------
        // Left Glyph servo
        //--------------------------------------------------------------------------
        try {
            v_glyph_servo_left = hardwareMap.servo.get(EngiNerdsConstants.GLYPH_SERVO_LEFT);
            //v_servo_top_claw.setDirection(Servo.Direction.REVERSE);
            v_glyph_servo_left.setPosition(Servo.MIN_POSITION);

        } catch (Exception p_exeception) {
            SetWarningMessage(EngiNerdsConstants.GLYPH_SERVO_LEFT);
            RobotLog.i(p_exeception.getLocalizedMessage());
            v_glyph_servo_left = null;
        }

        //--------------------------------------------------------------------------
        // Right Glyph servo
        //--------------------------------------------------------------------------
        try {
            v_glyph_servo_right = hardwareMap.servo.get(EngiNerdsConstants.GLYPH_SERVO_RIGHT);
            //v_servo_top_claw.setDirection(Servo.Direction.REVERSE);
            v_glyph_servo_right.setPosition(Servo.MAX_POSITION);

        } catch (Exception p_exeception) {
            SetWarningMessage(EngiNerdsConstants.GLYPH_SERVO_RIGHT);
            RobotLog.i(p_exeception.getLocalizedMessage());
            v_glyph_servo_right = null;
        }

        //--------------------------------------------------------------------------
        // RELIC TOP servo.
        //--------------------------------------------------------------------------
        try {
            v_relic_servo_top = hardwareMap.servo.get(EngiNerdsConstants.RELIC_SERVO_TOP);
            //v_servo_top_claw.setDirection(Servo.Direction.REVERSE);
            v_relic_servo_top.setPosition(0.7);

        } catch (Exception p_exeception) {
            SetWarningMessage(EngiNerdsConstants.RELIC_SERVO_TOP);
            RobotLog.i(p_exeception.getLocalizedMessage());
            v_relic_servo_top = null;
        }

        //--------------------------------------------------------------------------
        // RELIC BOTTOM servo.
        //--------------------------------------------------------------------------
        try {
            v_relic_servo_bottom = hardwareMap.servo.get(EngiNerdsConstants.RELIC_SERVO_BOTTOM);
            //v_servo_top_claw.setDirection(Servo.Direction.REVERSE);
            //v_relic_servo_bottom.setPosition(0.9);

        } catch (Exception p_exeception) {
            SetWarningMessage(EngiNerdsConstants.RELIC_SERVO_BOTTOM);
            RobotLog.i(p_exeception.getLocalizedMessage());
            v_relic_servo_bottom = null;
        }

        //--------------------------------------------------------------------------
        // Bottom Jewel servo.
        //--------------------------------------------------------------------------
        try {
            v_jewel_servo = hardwareMap.servo.get(EngiNerdsConstants.JEWEL_SERVO);
            v_jewel_servo.setPosition(0.0);

        } catch (Exception p_exeception) {
            SetWarningMessage(EngiNerdsConstants.JEWEL_SERVO);
            RobotLog.i(p_exeception.getLocalizedMessage());
            v_jewel_servo = null;
        }


        //--------------------------------------------------------------------------
        // Top Jewel Servo.
        //--------------------------------------------------------------------------
        try {
            v_jewel_servo_top = hardwareMap.servo.get(EngiNerdsConstants.JEWEL_SERVO_TOP);
            v_jewel_servo_top.setPosition(1.0);

        } catch (Exception p_exeception) {
            SetWarningMessage(EngiNerdsConstants.JEWEL_SERVO_TOP);
            RobotLog.i(p_exeception.getLocalizedMessage());
            v_jewel_servo_top = null;
        }

    }

    // init

    /**
     * Access whether a warning has been generated.
     */
    boolean isWarningGenerated() {
        return v_warning_generated;

    } // isWarningGenerated/

    /**
     * Access the warning message.
     */
    String GetWarningMessage() {
        return v_warning_message;

    } // GetWarningMessage

    //--------------------------------------------------------------------------
    // SetWarningMessage
    //

    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     * <p/>
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
    void SetWarningMessage(String p_exception_message) {
        if (v_warning_generated) {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    } // SetWarningMessage


    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    public float scaleMotorPower(float p_power) {
        //
        // Assume no scaling.
        //
        float l_scale = 0.0f;

        //
        // Ensure the values are legal.
        //
        float l_power = Range.clip(p_power, -1, 1);

        float[] l_array =
                {0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
                };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int) (l_power * 16.0);
        if (l_index < 0) {
            l_index = -l_index;
        } else if (l_index > 16) {
            l_index = 16;
        }

        if (l_power < 0) {
            l_scale = -l_array[l_index];
        } else {
            l_scale = l_array[l_index];
        }

        return l_scale;
    } // scaleMotorPower

    /**
     * Access the left drive motor's power level.
     */
    public double getLeftDrivePower() {
        double l_return = 0.0;

        if (v_motor_left_drive != null) {
            l_return = v_motor_left_drive.getPower();
        }

        return l_return;
    } // getLeftDrivePower

    /**
     * Access the right drive motor's power level.
     */
    public double getRightDrivePower() {
        double l_return = 0.0;

        if (v_motor_right_drive != null) {
            l_return = v_motor_right_drive.getPower();
        }

        return l_return;
    } // getRightDrivePower

    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    public void setDrivePower(double p_left_power, double p_right_power) {
        if (v_motor_left_drive != null) {
            v_motor_left_drive.setPower(p_left_power);
        }
        if (v_motor_right_drive != null) {
            v_motor_right_drive.setPower(p_right_power);
        }

    } // setDrivePower

    /**
     * Resets the drive encoder values.
     */
    public void resetDriveEncoder() {
        if (v_motor_left_drive != null) {
            v_motor_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (v_motor_right_drive != null) {
            v_motor_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * Resets the lift drive encoder values.
     */
    public void resetLiftEncoder() {
        if (v_motor_lift_drive != null) {
            v_motor_lift_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * Sets the mode to drive with encoder
     */
    public void setDriveWithEncoder() {
        if (v_motor_left_drive != null) {
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (v_motor_right_drive != null) {
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Sets the mode to drive with encoder
     */
    public void setDriveModeRunToPosition() {
        if (v_motor_left_drive != null) {
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (v_motor_right_drive != null) {
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Sets the mode to drive without encoder
     */
    public void setDriveWithoutEncoder() {
        if (v_motor_left_drive != null) {
            v_motor_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (v_motor_right_drive != null) {
            v_motor_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Sets the mode to drive with encoder
     */
    public void setLiftWithEncoder() {
        if (v_motor_lift_drive != null) {
            v_motor_lift_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setLiftModeRunToPosition() {
        if (v_motor_lift_drive != null) {
            v_motor_lift_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setLiftTargetPosition(int p_lift_count) {
        if (v_motor_lift_drive != null) {
            int targetCount = v_motor_lift_drive.getCurrentPosition() + p_lift_count;
            v_motor_lift_drive.setTargetPosition(targetCount);
        }
    }

    /**
     * Sets the mode to drive without encoder
     */
    public void setLiftWithoutEncoder() {
        if (v_motor_lift_drive != null) {
            v_motor_lift_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            v_motor_lift_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Sets the target position for the drive.
     *
     * @param p_left_count
     * @param p_right_count
     */
    public void setTargetPosition(int p_left_count, int p_right_count) {
        if (v_motor_left_drive != null) {
            int targetCount = v_motor_left_drive.getCurrentPosition() + p_left_count;
            v_motor_left_drive.setTargetPosition(targetCount);
        }
        if (v_motor_right_drive != null) {
            int targetCount = v_motor_right_drive.getCurrentPosition() + p_right_count;
            v_motor_right_drive.setTargetPosition(targetCount);
        }
    }

    /**
     * Set the Lift Motor power.
     */
    public void setLiftMotorPower(double lift_power) {
        if (v_motor_lift_drive != null) {
            v_motor_lift_drive.setPower(lift_power);
        }
    }

    /**
     * Set the Arm Motor power.
     */
    public void setArmMotorPower(double arm_power) {
        if (v_motor_arm_drive != null) {
            v_motor_arm_drive.setPower(arm_power);
        }
    }

    /**
     * Access the left encoder's count.
     */
    public int getLeftEncoderCount() {
        int l_return = 0;

        if (v_motor_left_drive != null) {
            l_return = v_motor_left_drive.getCurrentPosition();
        }
        return l_return;

    } // getLeftEncoderCount

    /**
     * Access the right encoder's count.
     */
    public int getRightEncoderCount() {
        int l_return = 0;
        if (v_motor_right_drive != null) {
            l_return = v_motor_right_drive.getCurrentPosition();
        }
        return l_return;
    }

    public void pickGylph() {
        v_glyph_servo_left.setPosition(0.4);
        v_glyph_servo_right.setPosition(0.6);
    }

    /**
     * Open Claw
     */
    public void openClaw() {
        v_glyph_servo_left.setPosition(Servo.MIN_POSITION);    // was 0.2 to less closing (now old servo)
        v_glyph_servo_right.setPosition(Servo.MAX_POSITION);
    }

    /**
     * Close Claw
     */
    public void closeClaw() {
        v_glyph_servo_left.setPosition(Servo.MAX_POSITION);    // was 0.8 (now  old servo)
        v_glyph_servo_right.setPosition(Servo.MIN_POSITION);   // was 0.2 (closing more)
    }

    /**
     * Partial Open
     */
    public void partialOpenClaw() {
        v_glyph_servo_left.setPosition(0.3);    // was 0.4 to less closing (now old servo)
        v_glyph_servo_right.setPosition(0.7);
    }

    // Top Relic Claw Close
    public void closeTopRelic() {
        v_relic_servo_top.setPosition(0.3);     // was 0.4

    }

    // Top Relic Claw Open
    public void openTopRelic() {
        v_relic_servo_top.setPosition(0.7);
    }


    // Bottom Relic Bottom
    public void closeBottomRelic() {
        v_relic_servo_bottom.setPosition(0.75); //was 0.8
    }

    public void releaseBottomRelic() {
        v_relic_servo_bottom.setPosition(0.6);  // was 0.7
        sleep(2000);
        v_relic_servo_bottom.setPosition(0.4);  // was 0.5
    }

    /**
     * Operates the lift motor
     *
     * @param direction - up or down
     * @param inches    - How many inches you want to move.
     */
    public void operateLift(String direction, double power, double inches) {
        //Calculate the encoder count per inch.
        long encoderCount = Math.round(inches * EngiNerdsConstants.LIFT_MOTOR_ENCODER_COUNT_PER_INCH);
        operateLiftWithEncoder(power, (int) encoderCount, 3);
    }

    /**
     * Method to perfmorm a relative move, based on encoder counts.
     * Encoders are not reset as the move is based on the current position.
     * Move will stop if any of three conditions occur:
     * 1) Move gets to the desired position
     * 2) Move runs out of time
     * 3) Driver stops the opmode running.
     */
    public void operateLiftWithEncoder(double power, int position, double timeoutSecs) {

        if (v_motor_lift_drive != null) {

            /*
            // Set Encoder Position
            v_motor_lift_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            v_motor_lift_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            v_motor_lift_drive.setTargetPosition(position);        //1120, 840, 560, 280

            // Reset Timer
            runtime.reset();
            v_motor_lift_drive.setPower(power);

            // Test if OpMode is running and Robot is moving
            while((runtime.seconds() < timeoutSecs) && v_motor_lift_drive.isBusy()){
                    sleep(250);
            }

            // End Movement and Reset
            v_motor_lift_drive.setPower(0.0);
            v_motor_lift_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            */


            //double increments = 0.005;
            //while ()
            //v_motor_lift_drive.setPower(lift_power);        //1120, 840, 560, 280


        }


    }

    /*
     * Sleep
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     *
     */
    public class Encoder {
        public int CurrentPositionModifier = 0;
        public int OldPosition = 0;

        protected DcMotor Motor;

        public Encoder(DcMotor motor) {
            this.Motor = motor;
        }

        public DcMotor getMotor() {
            return this.Motor;
        }

        public void setCurrentPosition(int position) {
            this.CurrentPositionModifier = position;
            this.OldPosition = this.Motor.getCurrentPosition();
        }

        public int getCurrentPosition() {
            return (this.Motor.getCurrentPosition() - this.OldPosition) + this.CurrentPositionModifier;
        }

        public void setTargetPosition(int target) {
            this.Motor.setTargetPosition((this.OldPosition + target) - this.CurrentPositionModifier);
        }

        public int getTargetPosition() {
            return (this.Motor.getTargetPosition() - this.OldPosition) + this.CurrentPositionModifier;
        }
    }

    /**
     * Jewel Arm Open
     */
    public void jewelArmOpen() {
        //Reset the Servo Position.
        v_jewel_servo_top.setPosition(0.25);
        waitFor(1);
        v_jewel_servo.setPosition(0.8);
        waitFor(1);
    }

    /**
     * Jewel Arm Close
     */
    public void jewelArmClose() {
        //Reset the Servo Position.
        v_jewel_servo.setPosition(0.0);
        waitFor(1);
        v_jewel_servo_top.setPosition(1.0);
        waitFor(1);
    }

    /**
     * Wait For Number of Seconds
     */
    private void waitFor(int seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds) {
            //telemetry.addData(">", "Waiting...: " + runtime.seconds());
            //telemetry.update();
            sleep(10);
        }
    }

    // Turn ON Brake Control
    public void setBrake() {
        v_motor_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v_motor_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Turn OFF Brake Control
    public void releaseBrake() {
        v_motor_right_drive.setZeroPowerBehavior(initialZPBehavior);
        v_motor_left_drive.setZeroPowerBehavior(initialZPBehavior);
    }

    // Raise or Lower the Lift (For Autonomous)
    public void raiseOrLowerLift(int encoderCount) {
        setLiftTargetPosition(encoderCount);  //One Rotation Up/Down
        setLiftModeRunToPosition();
        setLiftMotorPower(0.8);
    }

} // EngiNerdsHardware
