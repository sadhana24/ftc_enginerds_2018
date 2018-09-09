package org.firstinspires.ftc.enginerdsmec.hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//------------------------------------------------------------------------------
// EngiNerdsHardwareMec
//------------------------------------------------------------------------------

/**
 * Provides a single hardware access point between custom op-modes and the
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 */
public class EnginNerdsHardwareMec {

    private ElapsedTime runtime = new ElapsedTime();

    private boolean warningGenerated = false;
    private String warningMessage;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private OpMode opMode;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED             = 0.3;     // was 0.80   // Nominal speed for better accuracy.
    static final double TURN_SPEED              = 0.4;     // was 0.50   // Nominal half speed for better accuracy.
    static final double HEADING_THRESHOLD       = 4.0;     // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF            = 0.5;      // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    static final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder - 1440, NevRest 40 -1120
    static final double DRIVE_GEAR_REDUCTION    = 1.0 ;    // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DIGITAL_SERVO_CLOSE     = 0.7;
    static final double DIGITAL_SERVO_OPEN      = 0.3;

    static final double ANALOG_SERVO_CLOSE      = 0.7;
    static final double ANALOG_SERVO_OPEN       = 0.4;

    static final double DIGITAL_SERVO_CLOSE_FULL  = 0.9;
    static final double ANALOG_SERVO_CLOSE_FULL   = 1.0;


    // DC Motors
    public DcMotor leftFrontDrive;
    public DcMotor leftRearDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightRearDrive;
    public DcMotor relicArmDrive;
    public DcMotor glyphDrive;
    public DcMotor liftDrive;

    // Glyph Servos
    private Servo leftTopGlyphServo;
    private Servo leftBottomGlyphServo;
    private Servo rightTopGlyphServo;
    private Servo rightBottomGlyphServo;

    // Relic Servos
    private Servo topRelicServo;
    private Servo bottomRelicServo;

    // Jewel Servos
    private Servo jewelServo;
    private Servo jewelServoTop;
    private String GlyphPosition = "normal";

    // Sensors
    private ModernRoboticsI2cGyro gyroSensor;
    private DcMotor.ZeroPowerBehavior initialZeroPowerBehavior = DcMotor.ZeroPowerBehavior.UNKNOWN;

    private int glyphDriverEncoderValue = 0;

    //--------------------------------------------------------------------------
    // Initialize Hardware
    //--------------------------------------------------------------------------
    public void init(OpMode opMode, String mode) {

        //Op Mode
        this.opMode = opMode;

        // Telemetry
        this.telemetry = opMode.telemetry;

        // Hardware map
        this.hardwareMap = opMode.hardwareMap;

        warningGenerated = false;
        warningMessage = "Can't map; ";

        //--------------------------------------------------------------------------
        // Left Front Drive Motor
        //--------------------------------------------------------------------------
        try {
            leftFrontDrive = hardwareMap.dcMotor.get("left_front_drive");
            initialZeroPowerBehavior = leftFrontDrive.getZeroPowerBehavior();
        } catch (Exception p_exeception) {
            SetWarningMessage("coconut");
            RobotLog.i(p_exeception.getLocalizedMessage());
            leftFrontDrive = null;
        }

        //--------------------------------------------------------------------------
        // Left Rear Drive Motor
        //--------------------------------------------------------------------------
        try {
            leftRearDrive = hardwareMap.dcMotor.get("left_rear_drive");
        } catch (Exception p_exeception) {
            SetWarningMessage("left_rear_drive");
            RobotLog.i(p_exeception.getLocalizedMessage());
            leftRearDrive = null;
        }

        //--------------------------------------------------------------------------
        // Right Front Drive Motor
        //--------------------------------------------------------------------------
        try {
            rightFrontDrive = hardwareMap.dcMotor.get("right_front_drive");
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
         } catch (Exception p_exeception) {
            SetWarningMessage("right_front_drive");
            RobotLog.i(p_exeception.getLocalizedMessage());
            rightFrontDrive = null;
        }

        //--------------------------------------------------------------------------
        // Right Rear Drive Motor
        //--------------------------------------------------------------------------
        try {
            rightRearDrive = hardwareMap.dcMotor.get("right_rear_drive");
            rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception p_exeception) {
            SetWarningMessage("right_rear_drive");
            RobotLog.i(p_exeception.getLocalizedMessage());
            rightRearDrive = null;
        }

        //--------------------------------------------------------------------------
        // Relic Arm Drive Motor
        //--------------------------------------------------------------------------
        try {
            relicArmDrive = hardwareMap.dcMotor.get("relic_arm_drive");
            relicArmDrive.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception p_exeception) {
            SetWarningMessage("relic_arm_drive");
            RobotLog.i(p_exeception.getLocalizedMessage());
            relicArmDrive = null;
        }

        //--------------------------------------------------------------------------
        // Lift Drive Motor
        //--------------------------------------------------------------------------
        try {
            liftDrive = hardwareMap.dcMotor.get("lift_drive");
            liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Should prevent sliding down automatically.
        } catch (Exception p_exeception) {
            SetWarningMessage("lift_drive");
            RobotLog.i(p_exeception.getLocalizedMessage());
            liftDrive = null;
        }

        //--------------------------------------------------------------------------
        // Glyph Drive Motor
        //--------------------------------------------------------------------------
        try {
            glyphDrive = hardwareMap.dcMotor.get("glyph_drive");
            glyphDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception p_exeception) {
            SetWarningMessage("glyph_drive");
            RobotLog.i(p_exeception.getLocalizedMessage());
            glyphDrive = null;
        }

        //--------------------------------------------------------------------------
        // Left Top Glyph Servo
        //--------------------------------------------------------------------------
        try {
            leftTopGlyphServo = hardwareMap.servo.get("left_top_glyph_servo");
            leftTopGlyphServo.setDirection(Servo.Direction.REVERSE);
            //leftTopGlyphServo.setPosition(DIGITAL_SERVO_MAX);
        } catch (Exception p_exeception) {
            SetWarningMessage("left_top_glyph_servo");
            RobotLog.i(p_exeception.getLocalizedMessage());
            leftTopGlyphServo = null;
        }

        //--------------------------------------------------------------------------
        // Left Bottom Glyph Servo
        //--------------------------------------------------------------------------
        try {
            leftBottomGlyphServo = hardwareMap.servo.get("left_bottom_glyph_servo");
            //leftBottomGlyphServo.setPosition(DIGITAL_SERVO_MAX);
        } catch (Exception p_exeception) {
            SetWarningMessage("left_bottom_glyph_servo");
            RobotLog.i(p_exeception.getLocalizedMessage());
            leftBottomGlyphServo = null;
        }

        //--------------------------------------------------------------------------
        // Right Top Glyph Servo
        //--------------------------------------------------------------------------
        try {
            rightTopGlyphServo = hardwareMap.servo.get("right_top_glyph_servo");
            //rightTopGlyphServo.setPosition(DIGITAL_SERVO_MAX);
        } catch (Exception p_exeception) {
            SetWarningMessage("right_top_glyph_servo");
            RobotLog.i(p_exeception.getLocalizedMessage());
            rightTopGlyphServo = null;
        }

        //--------------------------------------------------------------------------
        // Right Bottom Glyph Servo
        //--------------------------------------------------------------------------
        try {
            rightBottomGlyphServo = hardwareMap.servo.get("right_bottom_glyph_servo");
            rightBottomGlyphServo.setDirection(Servo.Direction.REVERSE);
            //rightBottomGlyphServo.setPosition(DIGITAL_SERVO_MAX);
        } catch (Exception p_exeception) {
            SetWarningMessage("right_bottom_glyph_servo");
            RobotLog.i(p_exeception.getLocalizedMessage());
            rightBottomGlyphServo = null;
        }

        //--------------------------------------------------------------------------
        // Bottom Jewel servo.
        //--------------------------------------------------------------------------
        try {
            jewelServo = hardwareMap.servo.get("jewel_servo");
            //jewelServo.setPosition(0.0);
        } catch (Exception p_exeception) {
            SetWarningMessage("jewel_servo");
            RobotLog.i(p_exeception.getLocalizedMessage());
            jewelServo = null;
        }

        //--------------------------------------------------------------------------
        // Top Jewel Servo.
        //--------------------------------------------------------------------------
        try {
            jewelServoTop = hardwareMap.servo.get("jewel_servo_top");
            //jewelServoTop.setPosition(0.5);
        } catch (Exception p_exeception) {
            SetWarningMessage("jewel_servo_top");
            RobotLog.i(p_exeception.getLocalizedMessage());
            jewelServoTop = null;
        }

        //--------------------------------------------------------------------------
        // Top Relic servo.
        //--------------------------------------------------------------------------
        try {
            topRelicServo = hardwareMap.servo.get("relic_servo_top");
            topRelicServo.setPosition(0.7);
        } catch (Exception p_exeception) {
            SetWarningMessage("relic_servo_top");
            RobotLog.i(p_exeception.getLocalizedMessage());
            topRelicServo = null;
        }

        //--------------------------------------------------------------------------
        // Bottom Relic servo.
        //--------------------------------------------------------------------------
        try {
            bottomRelicServo = hardwareMap.servo.get("relic_servo_bottom");
            bottomRelicServo.setPosition(0.75);
        } catch (Exception p_exeception) {
            SetWarningMessage("relic_servo_bottom");
            RobotLog.i(p_exeception.getLocalizedMessage());
            bottomRelicServo = null;
        }

        //--------------------------------------------------------------------------
        // Initialize Op Mode based hardware controls
        //--------------------------------------------------------------------------
        switch (mode) {

            // Autonomous Mode Specific Hardware controls
            case "MODE_AUTO":
                //--------------------------------------------------------------------------
                // Get the gyro sensor device instance
                //--------------------------------------------------------------------------
                try {
                    gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro_sensor");
                } catch (Exception p_exeception) {
                    SetWarningMessage("gyro_sensor");
                    RobotLog.i(p_exeception.getLocalizedMessage());
                    gyroSensor = null;
                }

                if (liftDrive != null){
                    liftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                //Enable the encoders.
                setDriveMotorWithEncoder();
                break;



            // TeleOp Mode Specific Hardware controls
            case "MODE_TELEOP":
                if (liftDrive != null){
                    liftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                // Reset Drive Motor Encoders
                setDriveMotorWithoutEncoder();
            default:

                break;
        }



        // Reset Glyph Motor Encoder
        //resetGlyphMotorEncoder();

        // Set Glyph Motor Encoder
        //setGlyphMotorWithEncoder();

        // Close Glyph claws in controlled manner
        closeClawsControlled();

        // During the initialization close the Jewel arm
        jewelArmClose();
    }

    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     * <p/>
     * A comma will be added before the specified message if the message isn't
     * empty.
     *
     * @param exceptionMessage
     */
    void SetWarningMessage(String exceptionMessage) {
        if (warningGenerated) {
            warningMessage += ", ";
        }
        warningGenerated = true;
        warningMessage += exceptionMessage;

    } // SetWarningMessage

    /**
     * Scale the joystick input using a nonlinear algorithm.
     *
     * @param p_power
     * @return
     */
    public float scaleMotorPower(float p_power) {

        // Assume no scaling.
        float l_scale = 0.0f;

        // Ensure the values are legal.
        float l_power = Range.clip(p_power, -1, 1);

        float[] l_array =
                {0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
                };

        // Get the corresponding index for the specified argument/parameter.
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
     * Mecanum Drive
     *
     * @param speed
     * @param direction
     * @param rightX
     */
    public void mecanumDrive(double speed, double direction, double rightX) {

        final double v1 = speed * Math.cos(direction) - rightX;
        final double v2 = speed * Math.sin(direction) + rightX;
        final double v3 = speed * Math.sin(direction) - rightX;
        final double v4 = speed * Math.cos(direction) + rightX;

        //telemetry.addData("Drive Power Values > ", "V1: " + v1 + "; V2: " + v2 + "; V3: " + v3 + "; V4: " + v4);
        //telemetry.update();

        //logTelemetry("Drive Power Values > ", "V1: " + v1 + "; V2: " + v2 + "; V3: " + v3 + "; V4: " + v4);

        if(leftFrontDrive != null) {
            leftFrontDrive.setPower(v1);
        }
        if(rightFrontDrive != null) {
            rightFrontDrive.setPower(v2);
        }
        if(leftRearDrive != null) {
            leftRearDrive.setPower(v3);
        }
        if(rightRearDrive != null) {
            rightRearDrive.setPower(v4);
        }
    }

    public void mecanumDriveSimple(double xSpeed, double ySpeed){

        if (xSpeed == 0.0 && ySpeed == 0.0){
            if(leftFrontDrive != null) {
                leftFrontDrive.setPower(0.0);
            }
            if(rightFrontDrive != null) {
                rightFrontDrive.setPower(0.0);
            }
            if(leftRearDrive != null) {
                leftRearDrive.setPower(0.0);
            }
            if(rightRearDrive != null) {
                rightRearDrive.setPower(0.0);
            }

            return;
        }


        if (Math.abs(ySpeed) > Math.abs(xSpeed)){

            if(leftFrontDrive != null) {
                leftFrontDrive.setPower(ySpeed);
            }
            if(rightFrontDrive != null) {
                rightFrontDrive.setPower(ySpeed);
            }
            if(leftRearDrive != null) {
                leftRearDrive.setPower(ySpeed);
            }
            if(rightRearDrive != null) {
                rightRearDrive.setPower(ySpeed);
            }

        } else {

            if (leftFrontDrive != null) {
                leftFrontDrive.setPower(-1.0 * xSpeed);
            }
            if (rightFrontDrive != null) {
                rightFrontDrive.setPower(xSpeed);
            }
            if (leftRearDrive != null) {
                leftRearDrive.setPower(xSpeed);
            }
            if (rightRearDrive != null) {
                rightRearDrive.setPower(-1.0 * xSpeed);
            }
        }

    }

    public void mecanumDrive_Cartesian(double x, double y, double rotation)
    {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        normalizeSpeeds(wheelSpeeds);

        leftFrontDrive.setPower(wheelSpeeds[0]);
        rightFrontDrive.setPower(wheelSpeeds[1]);
        leftRearDrive.setPower(wheelSpeeds[2]);
        rightRearDrive.setPower(wheelSpeeds[3]);
    }   //mecanumDrive_Cartesian

    private void normalizeSpeeds(double[] wheelSpeeds)
    {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }   //normalize

    /**
     * Operate List Motor
     */
    public void operateLiftMotor(double lift_power) {
        if (liftDrive != null) {
            liftDrive.setPower(lift_power);
        }
    }

    /**
     * Jewel Arm Open
     */
    public void jewelArmOpen() {
        jewelServo.setPosition(0.55);
        waitFor(1);
        jewelServoTop.setPosition(0.8);
        waitFor(1);
    }

    /**
     * Jewel Arm Close
     */
    public void jewelArmClose() {
        jewelServoTop.setPosition(0.0);
        waitFor(1);
        jewelServo.setPosition(0.55);
        waitFor(1);
    }

    /**
     * Removes the jewel on the right.
     */
    public void removeJewelRight(){
        jewelServo.setPosition(0.3);
    }

    /**
     * Removes the jewel on the left.
     */
    public void removeJewelLeft(){
        jewelServo.setPosition(0.9);
    }

    public void gyroDriveSimple(double speed, double distance, double angle){
        int     moveCounts;

        // Ensure that the opmode is still active
        if (!isOpModeActive()) {
            return;
        }

        // Determine new target position, and pass to motor controller
        moveCounts = (int) (distance * COUNTS_PER_INCH);
        setTargetPosition(moveCounts, moveCounts);
        setDriveModeRunToPosition();

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 0.5);
        setDrivePower(speed, speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (isOpModeActive() && !isStopRequested() && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())
                && (rightFrontDrive.isBusy() && rightRearDrive.isBusy())) {
            idle();
        }

        setDrivePower(0, 0);

        setDriveMotorWithoutEncoder();

    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive (double speed, double distance, double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (isOpModeActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            setTargetPosition(moveCounts, moveCounts);
            setDriveModeRunToPosition();

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 0.5);
            setDrivePower(speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (isOpModeActive()&& !isStopRequested() && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())
                                    && (rightFrontDrive.isBusy() && rightRearDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 0.5)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                setDrivePower(leftSpeed, rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  String.format("%5.1f/%5.1f",  error, steer));
                telemetry.addData("Speed",   String.format("%5.2f:%5.2f",  leftSpeed, rightSpeed));
                telemetry.update();

                //this.log("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                log("Actual Encoder - Front",  String.format("%7d:%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition()));
                log("Actual Encoder - Back",  String.format("%7d:%7d", leftRearDrive.getCurrentPosition(), rightRearDrive.getCurrentPosition()));

            }

            // Stop all motion;
            setDrivePower(0.0, 0.0);

            // Reset Encoders
            resetDriveMotorEncoder();
            setDriveMotorWithoutEncoder();
        }

    } //gyroDrive

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (isOpModeActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            //telemetry.update();
        }

    } //gyroTurn

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (isOpModeActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            //telemetry.update();
        }

        // Stop all motion;
        setDrivePower(0.0, 0.0);

    } //gyroHold

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            //rightSpeed  = speed * steer;
            //leftSpeed   = -rightSpeed;
            leftSpeed  = speed * steer;
            rightSpeed   = -leftSpeed;
        }

        // Send desired speeds to motors.
        setDrivePower(leftSpeed, rightSpeed);

        // Display it for the driver.
        this.log("Target", String.format("%5.2f", angle));
        this.log("Err/St", String.format("%5.2f/%5.2f", error, steer));
        this.log("Speed.", String.format("%5.2f:%5.2f", leftSpeed, rightSpeed));
        this.log("Heading", "Initial Gyro Heading: " + gyroSensor.getIntegratedZValue());

        return onTarget;

    } //onHeading

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyroSensor.getIntegratedZValue();
        while(isOpModeActive() && robotError > 180) robotError -= 360;
        while(isOpModeActive() && robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * Sets the target position for the drive.
     *
     * @param leftTargetCount
     * @param rightTargetCount
     */
    public void setTargetPosition(int leftTargetCount, int rightTargetCount) {
        if (leftFrontDrive != null) {
            int targetCount = leftFrontDrive.getCurrentPosition() + leftTargetCount;
            leftFrontDrive.setTargetPosition(targetCount);
        }
        if (leftRearDrive != null) {
            int targetCount = leftRearDrive.getCurrentPosition() + leftTargetCount;
            leftRearDrive.setTargetPosition(targetCount);
        }
        if (rightFrontDrive != null) {
            int targetCount = rightFrontDrive.getCurrentPosition() + rightTargetCount;
            rightFrontDrive.setTargetPosition(targetCount);
        }
        if (rightRearDrive != null) {
            int targetCount = rightRearDrive.getCurrentPosition() + rightTargetCount;
            rightRearDrive.setTargetPosition(targetCount);
        }
    }

    /**
     * Sets the mode to drive with encoder
     */
    public void setDriveModeRunToPosition() {
        if (leftFrontDrive != null) {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (leftRearDrive != null) {
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (rightFrontDrive != null) {
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (rightRearDrive != null) {
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Sets the target position for the drive.
     *
     * @param leftPower
     * @param rightPower
     */
    public void setDrivePower(double leftPower, double rightPower) {
        if (leftFrontDrive != null) {
            leftFrontDrive.setPower(leftPower);
        }
        if (leftRearDrive != null) {
            leftRearDrive.setPower(leftPower);
        }

        if (rightFrontDrive != null) {
            rightFrontDrive.setPower(rightPower);
        }
        if (rightRearDrive != null) {
            rightRearDrive.setPower(rightPower);
        }

    }

    /**
     * Resets the drive motor encoder values.
     */
    public void resetDriveMotorEncoder() {
        if (leftFrontDrive != null) {
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (leftRearDrive != null) {
            leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (rightFrontDrive != null) {
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (rightRearDrive != null) {
            rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * Sets the mode to drive without encoder
     */
    public void setDriveMotorWithoutEncoder() {
        if (leftFrontDrive != null) {
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (leftRearDrive != null) {
            leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightFrontDrive != null) {
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightRearDrive != null) {
            rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Sets the mode to drive with encoder
     */
    public void setDriveMotorWithEncoder() {
        if (leftFrontDrive != null) {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (leftRearDrive != null) {
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (rightFrontDrive != null) {
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (rightRearDrive != null) {
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void RaiseLift(int encoderCount){
        if (liftDrive == null) return;
        this.logTelemetry("RaiseLift", String.format("Current Encoder Value:%7d" , liftDrive.getCurrentPosition()));
        liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftDrive.setTargetPosition(encoderCount);
        liftDrive.setPower(0.5);
        while (isOpModeActive() && liftDrive.isBusy()) {
            idle();
        }

    }

    public void LowerLift(int encoderCount){
        if (liftDrive == null) return;
        this.logTelemetry("LowerLift", String.format("Current Encoder Value:%7d" , liftDrive.getCurrentPosition()));
        liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftDrive.setTargetPosition(encoderCount);
        liftDrive.setPower(0.5);
        while (isOpModeActive() && liftDrive.isBusy()) {
            idle();
        }
    }

    /**
     * Resets the glyph motor encoder values.
     */
    public void resetGlyphMotorEncoder() {
        if (glyphDrive != null) {
            glyphDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * Sets the mode to glyph motor without encoder
     */
    public void setGlyphMotorWithoutEncoder() {
        if (glyphDrive != null) {
            glyphDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            glyphDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Sets the mode to glyph motor with encoder
     */
    public void setGlyphMotorWithEncoder() {
        if (glyphDrive != null) {
            glyphDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *
     */
    public void calibrateGyro() {
        gyroSensor.calibrate();
        while (isOpModeActive() && !isStopRequested() && gyroSensor.isCalibrating()) {
            sleep(50);
            idle();
        }
    }

    /**
     *
     */
    public void resetZAxisIntegrator() {
        gyroSensor.resetZAxisIntegrator();
    }

    /**
     * Wait For Number of Seconds
     *
     * @param seconds
     */
    private void waitFor(int seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds) {
            idle();
        }
    }

    /**
     * Sleep
     *
     * @param milliseconds
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Writes messages to log only
     *
     * @param caption
     * @param msg
     */
    public void log(String caption, String msg){
        RobotLog.ii("EngiNerds - " + caption, msg);
    }

    /**
     * Sends message to drive station(telemetry) only
     *
     * @param caption
     * @param msg
     */
    public void telemetry(String caption, String msg){
        telemetry.addData(caption, msg);
        telemetry.update();
    }

    /**
     * Writes messages to log and telemetry
     *
     * @param caption
     * @param msg
     */
    public void logTelemetry(String caption, String msg){
        RobotLog.ii("EngiNerds - " + caption, msg);
        telemetry.addData(caption, msg);
        telemetry.update();
    }

    /**
     * Use this mothod only for LinearOpMode case
     *
     * @return
     */
    public boolean isOpModeActive() {
        if(opMode instanceof LinearOpMode) {
            return ((LinearOpMode) opMode).opModeIsActive();
        } else {
            return false;
        }
    }

    /**
     * Use this mothod only for LinearOpMode case
     *
     * @return
     */
    public boolean isStopRequested() {
        if(opMode instanceof LinearOpMode) {
            return ((LinearOpMode) opMode).isStopRequested();
        } else {
            return false;
        }
    }

    /**
     * Use this mothod only for LinearOpMode case
     *
     * @return
     */
    public void idle() {
        if(opMode instanceof LinearOpMode) {
            ((LinearOpMode) opMode).idle();
        }
    }

    /**
     * Open Top and Bottom Glyph Claws
     */
    public void openTopAndBottomGlyphClaws() {
        openClaws("bottom");
        openClaws("top");
    }

    public void closeTopAndBottomGlyphClaws(){
        closeClaws("bottom");
        closeClaws("top");
    }

    /**
     * Opens the lower Glyph regardless of rotated or not.
     */
    public void wideOpenClaws(String position){
        if (position.equals("top")) {
            if (this.GlyphPosition.equals("normal")) {//Motor is rotated. Open the top ones
                rightTopGlyphServo.setPosition(0.0);
                leftTopGlyphServo.setPosition(0.0);
            } else {
                rightBottomGlyphServo.setPosition((0.0));
                leftBottomGlyphServo.setPosition(0.0);
            }
        }
        if (position.equals("bottom")) {
            if (this.GlyphPosition.equals("rotated")) {//Motor is rotated. Open the top ones
                rightTopGlyphServo.setPosition(0.0);
                leftTopGlyphServo.setPosition(0.0);
            } else {
                rightBottomGlyphServo.setPosition((0.0));
                leftBottomGlyphServo.setPosition(0.0);
            }
        }
    }


    /**
     * Opens the lower Glyph regardless of rotated or not.
     */
    public void openClaws(String position){
        if (position.equals("top")) {
            if (this.GlyphPosition.equals("normal")) {//Motor is rotated. Open the top ones
                openTopGlyphClaw();
            } else {
                openBottomGlyphClaw();
            }
        }
        if (position.equals("bottom")) {
            if (this.GlyphPosition.equals("rotated")) {//Motor is rotated. Open the top ones
                openTopGlyphClaw();
            } else {
                openBottomGlyphClaw();
            }
        }
    }

    /**
     * Closes the lower Glyph regardless of  rotated or not.
     */
    public void closeClaws(String position){
        if (position.equals("top")) {
            if (this.GlyphPosition.equals("normal")) {//Motor is NOT rotated. Open the top ones
                closeTopGlyphClaw();
            } else {
                closeBottomGlyphClaw();
            }
        }
        if(position.equals("bottom")) {
            if (this.GlyphPosition.equals("rotated")) {//Motor is rotated. Open the top ones
                closeTopGlyphClaw();
            } else {
                closeBottomGlyphClaw();
            }
        }
    }

    /**
     * Closes the Glyph servos in a controlled manner.
     */
    public void closeClawsControlled() {
        leftTopGlyphServo.setPosition(DIGITAL_SERVO_CLOSE_FULL);
        leftBottomGlyphServo.setPosition(ANALOG_SERVO_CLOSE_FULL);
        sleep(1000);
        rightTopGlyphServo.setPosition(DIGITAL_SERVO_CLOSE_FULL);
        rightBottomGlyphServo.setPosition((ANALOG_SERVO_CLOSE_FULL));
    }

    public void openTopGlyphClaw() {
        rightTopGlyphServo.setPosition(DIGITAL_SERVO_OPEN);
        leftTopGlyphServo.setPosition(DIGITAL_SERVO_OPEN);
    }

    public void closeTopGlyphClaw() {
        rightTopGlyphServo.setPosition(DIGITAL_SERVO_CLOSE);
        leftTopGlyphServo.setPosition(DIGITAL_SERVO_CLOSE);
    }

    public void openBottomGlyphClaw() {
        rightBottomGlyphServo.setPosition(ANALOG_SERVO_OPEN);
        leftBottomGlyphServo.setPosition(ANALOG_SERVO_OPEN);
    }

    public void closeBottomGlyphClaw() {
        rightBottomGlyphServo.setPosition((DIGITAL_SERVO_CLOSE));
        leftBottomGlyphServo.setPosition(DIGITAL_SERVO_CLOSE);
    }


    /**
     * Rotate Glyph Motor
     */
    public void rotateGlyphMotor(boolean direction) {
        logTelemetry("rotateGlyphMotor", "Current encoder count: " + glyphDrive.getCurrentPosition());

        //Do it in a simple way.
        if (direction){
            glyphDrive.setPower(0.3);
            this.GlyphPosition = "rotated";
        } else {
            glyphDrive.setPower(-0.3);
            this.GlyphPosition = "normal";
        }

        waitFor(1);
        glyphDrive.setPower(0.0);

        /*
        glyphDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (direction) {
            glyphDrive.setTargetPosition(575);
            glyphDrive.setPower(0.5);
        } else {
            glyphDrive.setTargetPosition(0);
            glyphDrive.setPower(0.5);
        }
        */
    }

    // Top Relic Claw Close
    public void openTopRelic() {
        topRelicServo.setPosition(0.3);     // was 0.4

    }

    // Top Relic Claw Open
    public void closeTopRelic() {
        topRelicServo.setPosition(0.7);
    }


    // Bottom Relic Bottom
    public void closeBottomRelic() {
        bottomRelicServo.setPosition(0.75); //was 0.8
    }

    public void releaseBottomRelic() {
        bottomRelicServo.setPosition(0.6);  // was 0.7
        waitFor(2);
        bottomRelicServo.setPosition(0.4);  // was 0.5
    }

    /**
     * Set the Arm Motor power.
     */
    public void setRelicArmDrivePower(double arm_power) {
        if (relicArmDrive != null) {
            relicArmDrive.setPower(arm_power);
        }
    }

    // Turn ON Brake Control
    public void setBrake() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Turn OFF Brake Control
    public void releaseBrake() {
        leftFrontDrive.setZeroPowerBehavior(initialZeroPowerBehavior);
        leftRearDrive.setZeroPowerBehavior(initialZeroPowerBehavior);
        rightFrontDrive.setZeroPowerBehavior(initialZeroPowerBehavior);
        rightRearDrive.setZeroPowerBehavior(initialZeroPowerBehavior);
    }

    public void closeJewelArm() {
        jewelServo.setPosition(0.0);
        sleep(1000);
        jewelServoTop.setPosition(0.4);
    }

    public void oepnJewelArm() {
        jewelServo.setPosition(0.5);
        sleep(1000);
        jewelServoTop.setPosition(0.4);
    }

    /**
     * Gyro Drive Testing
     */
    public void gyroDriveTest() {
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        logTelemetry("GyroDrive", " Drive 48\", 0.0 Heading");
        gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches

        // Stop
        setDrivePower(0, 0);
        sleep(2*1000);

        logTelemetry("GyroDrive", " Turn 90deg CCW, Hold 4secs");
        gyroTurn(TURN_SPEED, -90.0);         // Turn  CCW to -45 Degrees
        gyroHold(TURN_SPEED,  -90.0, 0.0);   // Hold  45 Deg heading for a 1/2 second

        // Stop
        setDrivePower(0, 0);
        sleep(2*1000);

        /*
        logTelemetry("GyroDrive", " Drive 24\" at 90deg");
        gyroDrive(DRIVE_SPEED, 24.0, -90.0);  // Drive FWD 12 inches at 45 degrees

        // Stop
        setDrivePower(0, 0);
        sleep(2*1000);

        logTelemetry("GyroDrive", " Turn 45deg CW, Hold 4secs");
        gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
        gyroHold( TURN_SPEED,  45.0, 4.0);    // Hold  45 Deg heading for a 1/2 second

        // Stop
        setDrivePower(0, 0);
        sleep(5*1000);

        logTelemetry("GyroDrive", " Turn CW to 0deg and hold 4sec");
        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        gyroHold( TURN_SPEED,   0.0, 4.0);    // Hold  0 Deg heading for a 1 second

        // Stop
        setDrivePower(0, 0);
        sleep(5*1000);

        logTelemetry("GyroDrive", " Drive back 48inches");
        gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches

        // Stop
        setDrivePower(0, 0);
        logTelemetry("Path", "Complete");
        */
    }

}
