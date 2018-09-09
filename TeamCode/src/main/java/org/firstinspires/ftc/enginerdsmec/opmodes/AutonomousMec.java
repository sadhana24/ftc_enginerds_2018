package org.firstinspires.ftc.enginerdsmec.opmodes;

import android.support.annotation.NonNull;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.enginerdsmec.hardware.EnginNerdsHardwareMec;
import org.firstinspires.ftc.enginerds.utilities.EngiNerdsConstants;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by thangap on 10/26/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Mec", group = "Autonomous")
public class AutonomousMec extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    protected String g_AllianceColor = EngiNerdsConstants.ALLIANCE_COLOR_RED;
    protected String g_DriverPosition = EngiNerdsConstants.DRIVER_POSITION_LEFT_SIMPLE;
    private EnginNerdsHardwareMec robot = new  EnginNerdsHardwareMec();
    private ModernRoboticsI2cGyro gyro;
    private ColorSensor jewelColorSensor;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.5;     //was 0.80   // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.4;     //was 0.50   // Nominal half speed for better accuracy.
    static final int        LIFT_DRIVE_COUNT        = 1120*2;      //Distance you want to lift the Glyph in the autonomous mode.

    // Control block of code execution.
    static final boolean   enableVuforia = true;

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            this.log(">", "===Autonomous Opmode START======================================================================================");
            // Send telemetry message to alert driver that we are calibrating;
            this.log(">", "Initializing Hardware Map");

            // Initialize robot hardware
            robot.init(this, "MODE_AUTO");

            robot.setBrake();

            // Get device instances
            this.log(">", "Getting device Instances");

            // Get the gyro sensor device instance
            gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get(EngiNerdsConstants.GYRO_SENSOR);
            jewelColorSensor = hardwareMap.get(ColorSensor.class, EngiNerdsConstants.JEWEL_COLOR_RANGE_SENSOR);

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
                idle();
            }


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

            //Raise the lift.
            this.log(">", "Raise Lift");
            robot.RaiseLift(LIFT_DRIVE_COUNT);

            // Wait for the start
            if (isStopRequested() || !opModeIsActive()) {
                return;
            }

            // Pickup the Gylph
            this.log(">", "Pickup Glyph");
            robot.wideOpenClaws("bottom");

            // Wait for the start
            if (isStopRequested() || !opModeIsActive()) {
                return;
            }

            //Raise the lift.
            this.log(">", "Lower Lift");
            robot.LowerLift(0);

            // Wait for the start
            if (isStopRequested() || !opModeIsActive()) {
                return;
            }

             // Pickup the Gylph
            this.log(">", "Pickup Glyph - Close Claws");
            robot.closeClaws("bottom");

            waitFor(1);

            // Wait for the start
            if (isStopRequested() || !opModeIsActive()) {
                return;
            }
             //Raise the lift.
            this.log(">", "Raise Lift");
            robot.RaiseLift((int)(LIFT_DRIVE_COUNT/1.5));

            // Wait for the start
            if (isStopRequested() || !opModeIsActive()) {
                return;
            }


            //Remove Jewel based on alliance color
            this.log(">", "Ready to remove Jewel");
            removeJewel();

            // Wait for the start
            if (isStopRequested() || !opModeIsActive()) {
                return;
            }

            //Turn Right 90deg
            this.log(">", "Alliance Color: " + g_AllianceColor);
            this.log(">", "Driver position: " + g_DriverPosition);
            this.log(">", "Initial Gyro Heading: " + gyro.getIntegratedZValue());

            if (g_DriverPosition.equals(EngiNerdsConstants.DRIVER_POSITION_LEFT_SIMPLE)) {
                //Simple
                NavigateToCryptoBoxLeftSimple(relicTemplate);

            } else if (g_DriverPosition.equals(EngiNerdsConstants.DRIVER_POSITION_LEFT_COMPLEX)) {
                // Complex Turn
                NavigateToCryptoBoxLeftComplex(relicTemplate);

            } else if (g_DriverPosition.equals(EngiNerdsConstants.DRIVER_POSITION_RIGHT_SIMPLE)) {
                // Simple Turn
                NavigateToCryptoBoxRightSimple(relicTemplate);

            } else if (g_DriverPosition.equals(EngiNerdsConstants.DRIVER_POSITION_RIGHT_COMPLEX)) {
                //Complex Turn
                NavigateToCryptoBoxRightComplex(relicTemplate);

            } else {
                this.log("Autonomous", "Driver Position UNKNOWN");
            }

            // Disable motor encoders on both Drive and Lift motors
            robot.setDriveMotorWithoutEncoder();
            robot.releaseBrake();

        } catch (Exception e){}
    }  //runOpMode



    /**
     *
     */
    private void NavigateToCryptoBoxLeftSimple(VuforiaTrackable relicTemplate) {

        // Drive Distance, aim for center column - default
        double forwardMoveInches = 34;
        double turnAngle = 90;

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
        robot.gyroDriveSimple(DRIVE_SPEED, forwardMoveInches, 0.0);

        this.log(">", "Turn RIGHT 90deg. Initial Heading:" + gyro.getIntegratedZValue() );
        robot.gyroTurn(TURN_SPEED, turnAngle);

        // Lower the Lift
        robot.LowerLift(0);

        //Drive to crypto box
        this.log(">", "Driving 15inches 90.0heading" );
        robot.gyroDriveSimple(DRIVE_SPEED, -15, turnAngle);

        //Release the Glyph.
        this.log(">", "Releasing Glyph" );
        robot.openClaws("bottom");

        //Move back
        this.log(">", "Moving Back 10inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, 10, turnAngle);

        this.log(">", "Closing Claws" );
        robot.closeClawsControlled();

        //Move back
        this.log(">", "Moving forward 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, -6, turnAngle);

    } // NavigateToCryptoBoxLeftSimple

    private void NavigateToCryptoBoxRightSimple(VuforiaTrackable relicTemplate) {
        // Identify vuMark
        double forwardMoveInches = 32; //Aim for center column - default
        double turnAngle = 90;

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
        robot.gyroDriveSimple(DRIVE_SPEED, -forwardMoveInches, 0.0);

        this.log(">", "Turn Left 85deg. Initial Heading:" + gyro.getIntegratedZValue() );
        robot.gyroTurn(TURN_SPEED, turnAngle);

        // Lower the Lift
        robot.LowerLift(0);      // 1/4 Rotation Down

        //Drive to crypto box
        this.log(">", "Driving 20inches 85.0heading" );
        robot.gyroDriveSimple(DRIVE_SPEED, -15, turnAngle);

        //Release the Glyph.
        this.log(">", "Releasing Glyph" );
        robot.openClaws("bottom");

        //Move back
        this.log(">", "Moving Back 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, 10, turnAngle);

        this.log(">", "Closing Claws" );
        robot.closeClawsControlled();

        //Move back
        this.log(">", "Moving forward 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, -6, turnAngle);

    } //NavigateToCryptoBoxRightSimple

    private void NavigateToCryptoBoxLeftComplex(VuforiaTrackable relicTemplate) {

        double forwardMoveInches = 24; //Aim for center column - default
        double turnAngle = 325;
        // Identify vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        if (relicTemplate != null){
            this.log(">", "Getting VuMark");

            vuMark = GetRelicRecoveryVuMark(relicTemplate);
            this.log(">", String.format("Found VuMark: %s", vuMark));

            //Drive straight out of Balancing stone.
            robot.gyroDriveSimple(DRIVE_SPEED, -forwardMoveInches, 0.0);

            this.log(">", "Turn Left 270deg" );

            switch(vuMark){
                case LEFT:
                    turnAngle = 345;
                    forwardMoveInches = 11;
                    break;

                case RIGHT:
                    turnAngle = 320;
                    forwardMoveInches = 24;
                    break;

                case CENTER:
                default:
                    turnAngle = 335;
                    forwardMoveInches = 14;
                    break;

            }

        }

        robot.gyroTurn(TURN_SPEED, turnAngle);

        //Raise the lift.
        this.log(">", "Lower Lift");
        robot.LowerLift(0);

        this.log(">", String.format("Move forward %5.2f inches",  forwardMoveInches) );
        robot.gyroDriveSimple(DRIVE_SPEED, -forwardMoveInches, turnAngle);

        //Release the Glyph.
        this.log(">", "Releasing Glyph" );
        robot.openClaws("bottom");

        //Move back
        this.log(">", "Moving Back 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, 10, 0);

        this.log(">", "Closing Claws" );
        robot.closeClawsControlled();

        //Move back
        this.log(">", "Moving forward 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, -8, 0);

    }

    private void NavigateToCryptoBoxRightComplex(VuforiaTrackable relicTemplate) {

        double forwardMoveInches = 24; //Aim for center column - default
        double turnAngle = 270;
        // Identify vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        if (relicTemplate != null){
            this.log(">", "Getting VuMark");

            vuMark = GetRelicRecoveryVuMark(relicTemplate);
            this.log(">", String.format("Found VuMark: %s", vuMark));

            //Drive straight out of Balancing stone.
            robot.gyroDriveSimple(DRIVE_SPEED, forwardMoveInches, 0.0);

            //this.log(">", "Turn Left 280deg" );
            //robot.gyroTurn(TURN_SPEED, 270.0);

            forwardMoveInches = 10; //initialize for Center

            switch(vuMark){
                case LEFT:
                    turnAngle = 255;
                    forwardMoveInches = 24;
                    break;

                case RIGHT:
                    turnAngle = 215;
                    forwardMoveInches = 11;
                    break;

                case CENTER:
                default:
                    turnAngle = 235;
                    forwardMoveInches = 14;
                    break;
            }
        }

        robot.gyroTurn(TURN_SPEED, turnAngle);

        //Raise the lift.
        this.log(">", "Lower Lift");
        robot.LowerLift(0);

        this.log(">", String.format("Move forward %5.2f inches",  forwardMoveInches) );
        robot.gyroDriveSimple(DRIVE_SPEED, -forwardMoveInches, turnAngle);

        //Release the Glyph.
        this.log(">", "Releasing Glyph" );
        robot.openClaws("bottom");

        //Move back
        this.log(">", "Moving Back 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, 10, turnAngle);

        this.log(">", "Closing Claws" );
        robot.closeClawsControlled();

        //Move back
        this.log(">", "Moving forward 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, -8, turnAngle);

    } //NavigateToCryptoBoxRightComplex



    /**
     * Complex turns on the left of the center crptobox.
     */
    private void NavigateToCryptoBoxLeftComplex_OLD(VuforiaTrackable relicTemplate) {

        double forwardMoveInches = 28; //Aim for center column - default

        // Identify vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        if (relicTemplate != null){
            this.log(">", "Getting VuMark");

            vuMark = GetRelicRecoveryVuMark(relicTemplate);
            this.log(">", String.format("Found VuMark: %s", vuMark));

            //Drive straight out of Balancing stone.
            robot.gyroDriveSimple(DRIVE_SPEED, -forwardMoveInches, 0.0);

            this.log(">", "Turn Left 270deg" );
            robot.gyroTurn(TURN_SPEED, 270.0);

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

        robot.gyroDriveSimple(DRIVE_SPEED, -forwardMoveInches, 270);

        this.log(">", "Turn Right 0deg" );
        robot.gyroTurn(TURN_SPEED, 0);

        // Lower the Lift
        robot.rotateGlyphMotor(false);      // 1/4 Rotation Down

        //Drive to crypto box
        this.log(">", "Driving 8inches 0.0heading" );
        robot.gyroDriveSimple(DRIVE_SPEED, -8, 0);

        //Release the Glyph.
        this.log(">", "Releasing Glyph" );
        robot.openClaws("bottom");

        //Move back
        this.log(">", "Moving Back 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, 5, 0);

        //Move back
        this.log(">", "Moving forward 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, -5, 0);

    } //NavigateToCryptoBoxLeftComplex

    /**
     *
     */
    private void NavigateToCryptoBoxRightComplex_OLD(VuforiaTrackable relicTemplate) {

        double forwardMoveInches = 26; //Aim for center column - default

        // Identify vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        if (relicTemplate != null){
            this.log(">", "Getting VuMark");

            vuMark = GetRelicRecoveryVuMark(relicTemplate);
            this.log(">", String.format("Found VuMark: %s", vuMark));

            //Drive straight out of Balancing stone.
            robot.gyroDriveSimple(DRIVE_SPEED, forwardMoveInches, 0.0);

            this.log(">", "Turn Left 280deg" );
            robot.gyroTurn(TURN_SPEED, 270.0);

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
        robot.gyroDriveSimple(DRIVE_SPEED, -forwardMoveInches, 270);


        this.log(">", "Turn Right 0deg" );
        robot.gyroTurn(TURN_SPEED, 180.0);

        // Lower the Lift
        robot.rotateGlyphMotor(false);      // 1/4 Rotation Down

        //Drive to crypto box
        this.log(">", "Driving 8inches 0.0heading" );
        robot.gyroDriveSimple(DRIVE_SPEED, -8, 180.0);

        //Release the Glyph.
        this.log(">", "Releasing Glyph" );
        robot.openClaws("bottom");

        //Move back
        this.log(">", "Moving Back 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, 5, 180.0);

        //Move back
        this.log(">", "Moving forward 5inches" );
        robot.gyroDriveSimple(DRIVE_SPEED, -5, 180.0);

    } //NavigateToCryptoBoxRightComplex

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
    private void removeJewel() {

        /*
         * COLOR SENSOR IS FACING BACKWARDS
         */
        if(isStopRequested() || !opModeIsActive() ){
            robot.jewelArmClose();
            return;
        }

        this.log("Status", "RemoveJewel: Opening Jewel Arm..");
        robot.jewelArmOpen();
        this.log("Status", "RemoveJewel: Arm Opened");

        waitFor(1);

        if (Math.abs(jewelColorSensor.red() - jewelColorSensor.blue()) < 7 ) {
            this.log("Status", "RemoveJewel: Could not Detect Color");
            this.log("Status", "RemoveJewel: Closing Jewel Arm..");
            robot.jewelArmClose();
            this.log("Status", "RemoveJewel: Arm Closed");
            return;
        }


        //Assume that the color sensor is mounted on sideways pointing front of the robot.
        if (g_AllianceColor == EngiNerdsConstants.ALLIANCE_COLOR_RED) {
            if (jewelColorSensor.blue() > jewelColorSensor.red()) { //Red color detected

                //Move forward and sense the color
                this.log("Status", "RemoveJewel: Sensed RED.  Removing Jewel on the RIGHT");
                robot.removeJewelRight();

            } else {

                //Move backward.
                this.log("Status", "RemoveJewel: Sensed BLUE.  Removing Jewel on the LEFT");
                robot.removeJewelLeft();
            }

        } else {
            if (jewelColorSensor.blue() > jewelColorSensor.red()) { //Red color detected

                //Move backward and sense the color
                this.log("Status", "RemoveJewel: Sensed RED.  Removing Jewel on the LEFT");
                robot.removeJewelLeft();

            } else {
                //Move forward.
                this.log("Status", "RemoveJewel: Sensed BLUE. Removing Jewel on the RIGHT");
                robot.removeJewelRight();

            }
        }

        waitFor(1);

        this.log("Status", "RemoveJewel: Closing Jewel Arm..");
        robot.jewelArmClose();
        this.log("Status", "RemoveJewel: Arm Closed");

    } //removeJewel


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


    private void log(String caption, String msg){
        RobotLog.ii("EngiNerds - " + caption, msg);
        telemetry.addData(caption, msg);
        telemetry.update();
    }

    private void waitFor(int seconds) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            idle();
        }
    }

    private void gyroDriveTest() {
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        this.log("robot.gyroDrive", " Drive 48\", 0.0 Heading");
        telemetry.update();
        robot.gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches

        robot.setDrivePower(0, 0);
        sleep(5*1000);

        this.log("robot.gyroDrive", " Turn 45deg CCW, Hold 4secs");
        telemetry.update();
        robot.gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        robot.gyroHold( TURN_SPEED,  -45.0, 4.0);    // Hold  45 Deg heading for a 1/2 second

        robot.setDrivePower(0, 0);
        sleep(5*1000);

        this.log("robot.gyroDrive", " Drive 12\" at 45deg");
        telemetry.update();
        robot.gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees

        robot.setDrivePower(0, 0);
        sleep(5*1000);

        this.log("robot.gyroDrive", " Turn 45deg CW, Hold 4secs");
        telemetry.update();
        robot.gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
        robot.gyroHold( TURN_SPEED,  45.0, 4.0);    // Hold  45 Deg heading for a 1/2 second

        robot.setDrivePower(0, 0);
        sleep(5*1000);

        this.log("robot.gyroDrive", " Turn CW to 0deg and hold 4sec");
        telemetry.update();
        robot.gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        robot.gyroHold( TURN_SPEED,   0.0, 4.0);    // Hold  0 Deg heading for a 1 second

        robot.setDrivePower(0, 0);
        sleep(5*1000);

        this.log("robot.gyroDrive", " Drive back 48inches");
        telemetry.update();
        robot.gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches

        robot.setDrivePower(0, 0);
        this.log("Path", "Complete");
        telemetry.update();
    }

}
