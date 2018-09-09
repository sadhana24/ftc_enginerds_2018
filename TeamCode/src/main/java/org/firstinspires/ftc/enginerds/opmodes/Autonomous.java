package org.firstinspires.ftc.enginerds.opmodes;

import android.support.annotation.NonNull;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.enginerds.hardware.EnginNerdsHardware;
import org.firstinspires.ftc.enginerds.utilities.EngiNerdsConstants;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by thangap on 10/26/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "EngiNerds Autonomous", group = "Autonomous")
@Disabled
public class Autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    protected String g_AllianceColor = EngiNerdsConstants.ALLIANCE_COLOR_RED;
    protected String g_DriverPosition = EngiNerdsConstants.DRIVER_POSITION_LEFT_SIMPLE;
    private EnginNerdsHardware robot = new EnginNerdsHardware();
    private ModernRoboticsI2cGyro gyro;
    private ColorSensor jewelColorSensor;
    private Servo jewelServo;
    private Servo jewelServoTop;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder - 1440, NevRest 40 -1120
    static final double     DRIVE_GEAR_REDUCTION    = 0.75 ;    // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.60;     //was 0.80   // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.40;     //was 0.50   // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 4;        // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;      // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static final double     JEWEL_DISTANCE          = 3.0;      // Distance robot moves to knock the jewel out
    static final int        LIFT_DRIVE_COUNT        = 500;      //Distance you want to lift the Glyph in the autonomous mode.

    // Control block of code execution.
    static final boolean   enableVuforia = true;

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            // Send telemetry message to alert driver that we are calibrating;
            this.log(">", "Initializing Hardware Map");

            // Initialize robot hardware
            robot.init(hardwareMap);
            robot.setBrake();

            // Get device instances
            this.log(">", "Getting device Instances");

            // Get the gyro sensor device instance
            gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get(EngiNerdsConstants.GYRO_SENSOR);

            // Get Jewel Servos device instances
            jewelServo = hardwareMap.servo.get(EngiNerdsConstants.JEWEL_SERVO);
            jewelServoTop = hardwareMap.servo.get(EngiNerdsConstants.JEWEL_SERVO_TOP);

            // Get Jewel Color Sensor device instance
            jewelColorSensor = hardwareMap.get(ColorSensor.class, EngiNerdsConstants.JEWEL_COLOR_RANGE_SENSOR);

            // Keep the Jewel arm in closed position
            jewelServo.setPosition(0.0);
            jewelServoTop.setPosition(1.0); //Close the Servo.

            // Pickup the Gylph
            this.log(">", "Pickup Glyph");
            //robot.closeClaw();

            VuforiaTrackable relicTemplate = null;

            if (enableVuforia) {
                this.log(">", "Vuforia Enabled");
                relicTemplate = InitializeVuforia();
            } else {
                this.log(">", "Vuforia Disabled");
            }

            // Wait until the gyro sensor calibration finishes.
            this.log(">", "Calibrating Gyro");    //

            gyro.calibrate();
            while (!isStopRequested() && gyro.isCalibrating()) {
                sleep(50);
                idle();
            }

            // Enable motor encoders on both Drive and Lift motors
            robot.resetDriveEncoder();
            robot.setDriveWithEncoder();

            robot.resetLiftEncoder();
            robot.setLiftWithEncoder();

            // Robot Actions
            this.log(">", "Robot Ready to START");

            // Wait for the game to start (Display Gyro value), and reset gyro before we move..
            while (!isStarted()) {
                idle();
            }

            gyro.resetZAxisIntegrator();

            // Wait for the start
            if (isStopRequested() || !opModeIsActive()) {
                return;
            }

            // Pickup the Gylph
            this.log(">", "Pickup Glyph");
            robot.closeClaw();

            // Wait 2 Secs
            waitFor(2);

            // Raise the Lift
            robot.raiseOrLowerLift(LIFT_DRIVE_COUNT);     // 1/4 Ration Up

            //Remove Jewel based on alliance color
            this.log(">", "Ready to remove Jewel");
            String movement = removeJewel();

            //Turn Right 90deg
            this.log(">", "Alliance Color: " + g_AllianceColor);
            this.log(">", "Driver position: " + g_DriverPosition);
            this.log(">", "Initial Gyro Heading: " + gyro.getIntegratedZValue());

            if (g_DriverPosition.equals(EngiNerdsConstants.DRIVER_POSITION_LEFT_SIMPLE)) {
                //Simple
                NavigateToCryptoBoxLeftSimple(relicTemplate, movement);

            } else if (g_DriverPosition.equals(EngiNerdsConstants.DRIVER_POSITION_LEFT_COMPLEX)) {
                // Complex Turn
                NavigateToCryptoBoxLeftComplex(relicTemplate, movement);

            } else if (g_DriverPosition.equals(EngiNerdsConstants.DRIVER_POSITION_RIGHT_SIMPLE)) {
                // Simple Turn
                NavigateToCryptoBoxRightSimple(relicTemplate, movement);

            } else if (g_DriverPosition.equals(EngiNerdsConstants.DRIVER_POSITION_RIGHT_COMPLEX)) {
                //Complex Turn
                NavigateToCryptoBoxRightComplex(relicTemplate, movement);

            } else {
                this.log("Autonomous", "Driver Position UNKNOWN");
            }

            // Lower the Lift
            robot.raiseOrLowerLift(-LIFT_DRIVE_COUNT);      // 1/4 Rotation Down

        /*
        robot.setLiftTargetPosition(-LIFT_DRIVE_COUNT);  // 1/4 Rotation Down
        robot.setLiftModeRunToPosition();
        robot.setLiftMotorPower(0.8);
        */

            // Disable motor encoders on both Drive and Lift motors
            robot.resetDriveEncoder();
            robot.setDriveWithoutEncoder();
            robot.resetLiftEncoder();
            robot.setLiftWithoutEncoder();

            robot.releaseBrake();

        } catch (Exception e){}
    }  //runOpMode

    /**
     *
     */
    private void NavigateToCryptoBoxRightComplex(VuforiaTrackable relicTemplate, String movement) {

        double forwardMoveInches = 26; //Aim for center column - default

        //if(movement.equals("Forward")) {
        //    forwardMoveInches += (JEWEL_DISTANCE + 1);
        //} else if (movement.equals("Backward")) {
          //  forwardMoveInches -= (JEWEL_DISTANCE - 1);
        //}

        // Identify vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        if (relicTemplate != null){
            this.log(">", "Getting VuMark");

            vuMark = GetRelicRecoveryVuMark(relicTemplate);
            this.log(">", String.format("Found VuMark: %s", vuMark));

            //Drive straight out of Balancing stone.
            gyroDrive(DRIVE_SPEED, -forwardMoveInches, 0.0);

            this.log(">", "Turn Left 280deg" );
            gyroTurn(TURN_SPEED, 280.0);

            forwardMoveInches = 10; //initialize for Center

            switch(vuMark){
                case LEFT:
                    forwardMoveInches = 18;
                    break;

                case RIGHT:
                    forwardMoveInches = 2;
                    break;

                case CENTER:
                    forwardMoveInches = 10;
                    break;
            }
        }

        this.log(">", String.format("Move forward %5.2f inches",  forwardMoveInches) );
        gyroDrive(DRIVE_SPEED, forwardMoveInches, 280);

        //if(true) {
        //    return;
        //}

        this.log(">", "Turn Right 0deg" );
        gyroTurn(TURN_SPEED, 190.0);

        // Lower the Lift
        robot.raiseOrLowerLift(-LIFT_DRIVE_COUNT);      // 1/4 Rotation Down

        //Drive to crypto box
        this.log(">", "Driving 8inches 0.0heading" );
        gyroDrive(DRIVE_SPEED, 8, 190.0);

        //Release the Glyph.
        this.log(">", "Releasing Glyph" );
        robot.openClaw();

        //Move back
        this.log(">", "Moving Back 5inches" );
        gyroDrive(DRIVE_SPEED, -5, 190.0);

        //Move back
        this.log(">", "Moving forward 5inches" );
        gyroDrive(DRIVE_SPEED, 5, 190.0);

    } //NavigateToCryptoBoxRightComplex


    /**
     *
     */
    private void NavigateToCryptoBoxLeftSimple(VuforiaTrackable relicTemplate, String movement) {

        // Drive Distance, aim for center column - default
        double forwardMoveInches = 32;

        // Adjust the distance based on the distance moved during knocking the Jewel
       /* if(movement.equals("Forward")) {
            forwardMoveInches += (JEWEL_DISTANCE + 1);
        } else if (movement.equals("Backward")) {
            forwardMoveInches -= (JEWEL_DISTANCE - 1);
        }
        */
        // Identify vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        if (relicTemplate != null){
            this.log(">", "Getting VuMark");

            vuMark = GetRelicRecoveryVuMark(relicTemplate);

            this.log(">", String.format("Found VuMark: %s", vuMark));
            switch(vuMark){
                case LEFT:
                    forwardMoveInches += 8;
                    break;

                case RIGHT:
                    forwardMoveInches -= 8;
                    break;

                case CENTER:
                    forwardMoveInches += 0;
                    break;
            }
        }

        this.log(">", String.format("Driving out of Balancing Stone %5.2f inches",  forwardMoveInches) );

        //Drive straight out of Balancing stone. Robot is moving backwards and that's why the negative distance.
        gyroDrive(DRIVE_SPEED, -forwardMoveInches, 0.0);

        this.log(">", "Turn RIGHT 85deg. Initial Heading:" + gyro.getIntegratedZValue() );
        gyroTurn(TURN_SPEED, 85.0);

        // Lower the Lift
        robot.raiseOrLowerLift(-LIFT_DRIVE_COUNT);      // 1/4 Rotation Down

        //Drive to crypto box
        this.log(">", "Driving 15inches 85.0heading" );
        gyroDrive(DRIVE_SPEED, 15, 85.0);

        //Release the Glyph.
        this.log(">", "Releasing Glyph" );
        robot.openClaw();

        //Move back
        this.log(">", "Moving Back 10inches" );
        gyroDrive(DRIVE_SPEED, -10, 85.0);

        //Move back
        this.log(">", "Moving forward 5inches" );
        gyroDrive(DRIVE_SPEED, 5, 85.0);

    } // NavigateToCryptoBoxLeftSimple

    /**
     * This is handle the small distance Balancing stone on the right.
     */
    private void NavigateToCryptoBoxRightSimple(VuforiaTrackable relicTemplate, String movement) {
        // Identify vuMark
        double forwardMoveInches = 32; //Aim for center column - default

        //The Robot front is towards the crypto box.
       /* if(movement.equals("Forward")) {
            forwardMoveInches -= (JEWEL_DISTANCE - 1);
        } else if (movement.equals("Backward")) {
            forwardMoveInches += (JEWEL_DISTANCE + 1);
        }
        */
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        if (relicTemplate != null){
            this.log(">", "Getting VuMark");

            vuMark = GetRelicRecoveryVuMark(relicTemplate);

            this.log(">", String.format("Found VuMark: %s", vuMark));
            switch(vuMark){
                case LEFT:
                    forwardMoveInches -= 8;
                    break;

                case RIGHT:
                    forwardMoveInches += 8;
                    break;

                case CENTER:
                    forwardMoveInches += 0;
                    break;

            }

        }

        this.log(">", String.format("Driving out of Balancing Stone %5.2f inches",  forwardMoveInches) );

        //Drive straight out of Balancing stone.
        gyroDrive(DRIVE_SPEED, forwardMoveInches, 0.0);

        this.log(">", "Turn Left 85deg. Initial Heading:" + gyro.getIntegratedZValue() );
        gyroTurn(TURN_SPEED, 85.0);

        // Lower the Lift
        robot.raiseOrLowerLift(-LIFT_DRIVE_COUNT);      // 1/4 Rotation Down

        //Drive to crypto box
        this.log(">", "Driving 20inches 85.0heading" );
        gyroDrive(DRIVE_SPEED, 20, 85.0);

        //Release the Glyph.
        this.log(">", "Releasing Glyph" );
        robot.openClaw();

        //Move back
        this.log(">", "Moving Back 5inches" );
        gyroDrive(DRIVE_SPEED, -5, 85.0);

        //Move back
        this.log(">", "Moving forward 5inches" );
        gyroDrive(DRIVE_SPEED, 5, 85.0);

    } //NavigateToCryptoBoxRightSimple

    /**
     * Complex turns on the left of the center crptobox.
     */
    private void NavigateToCryptoBoxLeftComplex(VuforiaTrackable relicTemplate, String movement) {

        double forwardMoveInches = 28; //Aim for center column - default

        /*if(movement.equals("Forward")) {
            forwardMoveInches -= JEWEL_DISTANCE;
        } else if (movement.equals("Backward")) {
            forwardMoveInches += JEWEL_DISTANCE;
        }
    */

        // Identify vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        if (relicTemplate != null){
            this.log(">", "Getting VuMark");

            vuMark = GetRelicRecoveryVuMark(relicTemplate);
            this.log(">", String.format("Found VuMark: %s", vuMark));

            //Drive straight out of Balancing stone.
            gyroDrive(DRIVE_SPEED, forwardMoveInches, 0.0);

            this.log(">", "Turn Left 280deg" );
            gyroTurn(TURN_SPEED, 280.0);

            switch(vuMark){
                case LEFT:
                    forwardMoveInches = 0;
                    break;

                case RIGHT:
                    forwardMoveInches = 16;
                    break;

                case CENTER:
                    forwardMoveInches = 8;
                    break;

            }

        }

        this.log(">", String.format("Move forward %5.2f inches",  forwardMoveInches) );

        gyroDrive(DRIVE_SPEED, forwardMoveInches, 280);

        this.log(">", "Turn Right 0deg" );
        gyroTurn(TURN_SPEED, 180.0);

        // Lower the Lift
        robot.raiseOrLowerLift(-LIFT_DRIVE_COUNT);      // 1/4 Rotation Down

        //Drive to crypto box
        this.log(">", "Driving 8inches 0.0heading" );
        gyroDrive(DRIVE_SPEED, 8, 180.0);

        //Release the Glyph.
        this.log(">", "Releasing Glyph" );
        robot.openClaw();

        //Move back
        this.log(">", "Moving Back 5inches" );
        gyroDrive(DRIVE_SPEED, -5, 180.0);

        //Move back
        this.log(">", "Moving forward 5inches" );
        gyroDrive(DRIVE_SPEED, 5, 180.0);

    } //NavigateToCryptoBoxLeftComplex

    /**
     *
     */
    @NonNull
    private VuforiaTrackable InitializeVuforia() {
        VuforiaTrackable relicTemplate;
        this.log(">", "Initializing Vuforia..");

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = EngiNerdsConstants.VUFORIA_LICENSE_KEY;

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //This may have to be moved after start.
        relicTrackables.activate();


        this.log(">", "Initializing Vuforia...DONE");

        return relicTemplate;

    } //InitializeVuforia

    /**
     *
     */
    //Simple color detection and removal
    private String removeJewel() {

        /*
         * COLOR SENSOR IS FACING BACKWARDS
         */
        String movement = null;

        this.log("Status", "RemoveJewel: Opening Jewel Arm..");
        robot.jewelArmOpen();
        this.log("Status", "RemoveJewel: Arm Opened");

        waitFor(2);

        if(isStopRequested() || !opModeIsActive() ){
            jewelServo.setPosition(0.0);
            jewelServoTop.setPosition(1.0);
            return "NotReady";
        }

        if (Math.abs(jewelColorSensor.red() - jewelColorSensor.blue()) < 7 ) {
            this.log("Status", "RemoveJewel: Could not Detect Color");
            this.log("Status", "RemoveJewel: Closing Jewel Arm..");
            robot.jewelArmClose();
            this.log("Status", "RemoveJewel: Arm Closed");
            return "NotFound";
        }

        double jewelRemoveSpeed = DRIVE_SPEED/3.0;

        //Assume that the color sensor is mounted on sideways pointing back of the robot.
        if (g_AllianceColor == EngiNerdsConstants.ALLIANCE_COLOR_RED) {
            if (jewelColorSensor.blue() > jewelColorSensor.red()) { //Red color detected

                //Move forward and sense the color
                this.log("Status", "RemoveJewel: Sensed RED");    //
                this.log("Status", "RemoveJewel: Moving Forward");    //

             //   gyroDrive(jewelRemoveSpeed, JEWEL_DISTANCE, 0.0);    //Drive FWD 3 inches
                gyroTurn(jewelRemoveSpeed,340.00);
                movement = "Forward";

            } else {

                //Move backward.
                this.log("Status", "RemoveJewel: Sensed BLUE");    //
                this.log("Status", "RemoveJewel: Moving Backward");    //

               // gyroDrive(jewelRemoveSpeed, -JEWEL_DISTANCE, 0.0);    // Drive BWD 3 inches
                gyroTurn(jewelRemoveSpeed,20.00);
                movement = "Backward";
            }

        } else {
            if (jewelColorSensor.blue() > jewelColorSensor.red()) { //Red color detected

                //Move backward and sense the color
                this.log("Status", "RemoveJewel: Sensed RED");    //
                this.log("Status", "RemoveJewel: Moving Backward");    //

              //  gyroDrive(jewelRemoveSpeed, -JEWEL_DISTANCE, 0.0);    // Drive BWD 3 inches
                gyroTurn(jewelRemoveSpeed,20.00);
                movement = "Backward";

            } else {
                //Move forward.
                this.log("Status", "RemoveJewel: Sensed BLUE");    //
                this.log("Status", "RemoveJewel: Moving Forward");    //

                //gyroDrive(jewelRemoveSpeed, JEWEL_DISTANCE, 0.0);    // Drive FWD 3 inches
                gyroTurn(jewelRemoveSpeed,340.00);
                movement = "Forward";
            }
        }

        this.log("Status", "RemoveJewel: Closing Jewel Arm..");
        robot.jewelArmClose();
        this.log("Status", "RemoveJewel: Arm Closed");

        gyroTurn(jewelRemoveSpeed,0.00);
        return movement;

    } //removeJewel

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
    public void gyroDrive ( double speed, double distance, double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            robot.setTargetPosition(moveCounts, moveCounts);
            robot.setDriveModeRunToPosition();

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.v_motor_left_drive.setPower(speed);
            robot.v_motor_right_drive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.v_motor_left_drive.isBusy() && robot.v_motor_right_drive.isBusy())) {

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
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.v_motor_left_drive.setPower(leftSpeed);
                robot.v_motor_right_drive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  String.format("%5.1f/%5.1f",  error, steer));
                //this.log("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                this.log("Actual Encoder",  String.format("%7d:%7d",      robot.v_motor_left_drive.getCurrentPosition(), robot.v_motor_right_drive.getCurrentPosition()));
                telemetry.addData("Speed",   String.format("%5.2f:%5.2f",  leftSpeed, rightSpeed));
                telemetry.update();
            }

            // Stop all motion;
            robot.v_motor_left_drive.setPower(0);
            robot.v_motor_right_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
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
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.v_motor_left_drive.setPower(0);
        robot.v_motor_right_drive.setPower(0);

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
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.v_motor_left_drive.setPower(leftSpeed);
        robot.v_motor_right_drive.setPower(rightSpeed);

        // Display it for the driver.
        this.log("Target", String.format("%5.2f", angle));
        this.log("Err/St", String.format("%5.2f/%5.2f", error, steer));
        this.log("Speed.", String.format("%5.2f:%5.2f", leftSpeed, rightSpeed));
        this.log("Heading", "Initial Gyro Heading: " + gyro.getIntegratedZValue());

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
        robotError = targetAngle - gyro.getIntegratedZValue();
        while((opModeIsActive() && robotError > 180)) robotError -= 360;
        while((opModeIsActive() && robotError <= -180)) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /*
     *
     */
    private void DriveToPosition(int encoderCount, double power) {
        //Set the motors to run with encoder.
        robot.setDriveWithEncoder();

        //TODO: Set the target positions. Should be used in a loop
        //while ((opModeIsActive())) {
        robot.setTargetPosition(encoderCount, encoderCount);
        robot.setDrivePower(power, power);
        //}
        //Set the motors to run without encoder.
        robot.setDriveWithoutEncoder();
        sleep(500);
    }

    public void TurnRight90() throws InterruptedException {
        while ((opModeIsActive())) {
            if (gyro.getHeading() > 75 && gyro.getHeading() < 350) {
                robot.setDrivePower(0.0, 0.0);
                break;
            } else {
                robot.setDrivePower(0.5, 0.0);
            }
        }
        gyro.resetZAxisIntegrator();
    }

    public void TurnLeft45() throws InterruptedException {
        while (opModeIsActive()) {
            if (gyro.getHeading() < 328 && gyro.getHeading() > 10) {
                robot.setDrivePower(0.0, 0.0);
                break;
            } else {
                robot.setDrivePower(0.0, 0.5);
            }
        }
        gyro.resetZAxisIntegrator();
    }

    public void TurnRight45() throws InterruptedException {
        while (opModeIsActive()) {
            if (gyro.getHeading() > 32 && gyro.getHeading() < 350) {
                robot.setDrivePower(0.0, 0.0);
                break;
            } else {
                robot.setDrivePower(0.5, 0.0);
            }
        }
        gyro.resetZAxisIntegrator();
    }

    public void SpinRight90() throws InterruptedException {
        while ((opModeIsActive())) {
            if (gyro.getHeading() > 75 && gyro.getHeading() < 350) {
                robot.setDrivePower(0.0, 0.0);
                break;
            } else {
                robot.setDrivePower(-0.5, 0.5);
            }
        }
        gyro.resetZAxisIntegrator();
    }

    public void SpinLeft90() throws InterruptedException {
        while (opModeIsActive()) {
            if (gyro.getHeading() < 290 && gyro.getHeading() > 10) {
                robot.setDrivePower(0.0, 0.0);
                break;
            } else {
                robot.setDrivePower(0.5, -0.5);
            }
        }
        gyro.resetZAxisIntegrator();
    }

    public void SpinLeft45() throws InterruptedException {
        while (opModeIsActive()) {
            if (gyro.getHeading() < 332 && gyro.getHeading() > 10) {
                robot.setDrivePower(0.0, 0.0);
                break;
            } else {
                robot.setDrivePower(-0.5, 0.5);
            }
        }
        gyro.resetZAxisIntegrator();
    }

    public void SpinRight45() throws InterruptedException {
        while (opModeIsActive()) {
            if (gyro.getHeading() < 350 && gyro.getHeading() > 32) {
                robot.setDrivePower(0.0, 0.0);
                break;
            } else {
                robot.setDrivePower(0.5, -0.5);
            }
        }
        gyro.resetZAxisIntegrator();
    }

    public RelicRecoveryVuMark GetRelicRecoveryVuMark(VuforiaTrackable relicTemplate){

        RelicRecoveryVuMark detectedVurMark = RelicRecoveryVuMark.UNKNOWN;

        int forwardInches = 4;

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            detectedVurMark = vuMark;
            this.log("VuMark", String.format("%s visible", vuMark));
            break;

        }

        return detectedVurMark;

    }

    private String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    private void log(String caption, String msg){
        RobotLog.ii("EngiNerds - " + caption, msg);
        telemetry.addData(caption, msg);
        telemetry.update();
    }

    private void waitFor(int seconds) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData(">", "Waiting...: " + runtime.seconds());
            telemetry.update();
            sleep(10);
        }
    }

    private void GyroDriveTest() {
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        this.log("GyroDrive", " Drive 48\", 0.0 Heading");
        telemetry.update();
        gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches

        robot.setDrivePower(0, 0);
        sleep(5*1000);

        this.log("GyroDrive", " Turn 45deg CCW, Hold 4secs");
        telemetry.update();
        gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        gyroHold( TURN_SPEED,  -45.0, 4.0);    // Hold  45 Deg heading for a 1/2 second

        robot.setDrivePower(0, 0);
        sleep(5*1000);

        this.log("GyroDrive", " Drive 12\" at 45deg");
        telemetry.update();
        gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees

        robot.setDrivePower(0, 0);
        sleep(5*1000);

        this.log("GyroDrive", " Turn 45deg CW, Hold 4secs");
        telemetry.update();
        gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
        gyroHold( TURN_SPEED,  45.0, 4.0);    // Hold  45 Deg heading for a 1/2 second

        robot.setDrivePower(0, 0);
        sleep(5*1000);

        this.log("GyroDrive", " Turn CW to 0deg and hold 4sec");
        telemetry.update();
        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        gyroHold( TURN_SPEED,   0.0, 4.0);    // Hold  0 Deg heading for a 1 second

        robot.setDrivePower(0, 0);
        sleep(5*1000);

        this.log("GyroDrive", " Drive back 48inches");
        telemetry.update();
        gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches

        robot.setDrivePower(0, 0);
        this.log("Path", "Complete");
        telemetry.update();
    }

}
