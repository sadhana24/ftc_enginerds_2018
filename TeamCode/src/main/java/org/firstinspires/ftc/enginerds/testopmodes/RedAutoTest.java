package org.firstinspires.ftc.enginerds.testopmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
/*
Sample auto mode using sensors for navigation.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedAutoTest", group = "What is this")
@Disabled
public class RedAutoTest extends LinearOpMode {
    DcMotor leftWheel;
    DcMotor rightWheel;

    Servo buttonPusher;

    GyroSensor gyro;
    ModernRoboticsI2cRangeSensor frontRange;

    ColorSensor floor;
    ColorSensor beacon;

    OpticalDistanceSensor leftODS;
    OpticalDistanceSensor rightODS;

    int targetheading;

    int i = 1;
    int i1 = 1;
    int i2 = 1;
    int i3 = 1;
    int i4 = 1;
    int i5 = 1;
    int i6 = 1;
    int i7 = 1;
    int i8 = 1;
    int i9 = 1;
    int i10 = 1;
    int i11 = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        leftWheel = hardwareMap.dcMotor.get("lw");
        rightWheel = hardwareMap.dcMotor.get("rw");
        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        buttonPusher = hardwareMap.servo.get("b");

        buttonPusher.setPosition(.43);

        gyro = hardwareMap.gyroSensor.get("g");
        gyro.calibrate();

        frontRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range2");

        floor = hardwareMap.colorSensor.get("c");
        floor.setI2cAddress(I2cAddr.create8bit(0x3c));

        beacon = hardwareMap.colorSensor.get("c2");
        beacon.setI2cAddress(I2cAddr.create8bit(0x6c));

        leftODS = hardwareMap.opticalDistanceSensor.get("l");
        rightODS = hardwareMap.opticalDistanceSensor.get("r");

        telemetry.addData("CM", frontRange.cmUltrasonic());
        telemetry.addData("Blue", beacon.blue());
        telemetry.addData("Red", beacon.red());
        telemetry.addData("Green", floor.green());
        telemetry.addData("leftLight", leftODS.getLightDetected());
        telemetry.addData("rightLight", rightODS.getLightDetected());
        telemetry.update();

        waitForStart();

        while(gyro.isCalibrating()) {
            sleep(50);
        }

        turnLeft45(); //2
        sleep(100);
        encoderWhiteLine(); //3
        sleep(200);
        turnLeftPivot45(); //4
        sleep(200);
        parkBeacon(); //5
        sleep(200);
        redBeacon(); //good until this point
        sleep(200);
        backUp();
        sleep(200);
        turnRedBeacon();
        sleep(200);
        straightUntilWhite2(); //9
        sleep(300);
        turnLeft90(); //10
        sleep(200);
        parkBeacon2(); //11
        sleep(200);
        redBeacon2(); //12
    }

    public void encoderWhiteLine() throws InterruptedException {
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setTargetPosition(10);
        leftWheel.setPower(.15);
        rightWheel.setPower(.15);
        while(opModeIsActive() && i8 == 1) {
            if(floor.green() > 4) {
                leftWheel.setPower(0);
                rightWheel.setPower(0);
                i--;
            } else {
                rightWheel.setTargetPosition(leftWheel.getCurrentPosition() - 10);
                leftWheel.setTargetPosition(rightWheel.getCurrentPosition() - 10);
            }
        }
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(500);
        i8++;
    }

    public void turnLeft90() throws InterruptedException {
        while (opModeIsActive() && (i == 1)) {
            if (gyro.getHeading() < 290 && gyro.getHeading() > 10) {
                leftWheel.setPower(0);
                rightWheel.setPower(0);
                i--;
            } else {
                rightWheel.setPower(-.22);
                leftWheel.setPower(.22);
            }
        }
        i++;
        gyro.resetZAxisIntegrator();
    }

    public void turnRight90() throws InterruptedException {
        while ((opModeIsActive() && i1 == 1)) {
            if (gyro.getHeading() > 75 && gyro.getHeading() < 350) {
                leftWheel.setPower(0);
                rightWheel.setPower(0);
                i1--;
            } else {
                leftWheel.setPower(-.22);
                rightWheel.setPower(.22);
            }
        }
        i1++;
        gyro.resetZAxisIntegrator();
    }

    public void turnLeftPivot45() throws InterruptedException {
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && (i3 == 1)) {
            if (gyro.getHeading() < 332 && gyro.getHeading() > 10) {
                leftWheel.setPower(0);
                rightWheel.setPower(0);
                i3--;
            } else {
                rightWheel.setPower(-.15);
                leftWheel.setPower(.15);
            }
        }
        i3++;
        targetheading = gyro.getHeading() - 235;
        gyro.resetZAxisIntegrator();
    }

    public void turnRightPivot45() throws InterruptedException {
        while (opModeIsActive() && (i7 == 1)) {
            if (gyro.getHeading() < 350 && gyro.getHeading() > 32) {
                leftWheel.setPower(0);
                rightWheel.setPower(0);
                i7--;
            } else {
                rightWheel.setPower(.15);
                leftWheel.setPower(-.15);
            }
        }
        i7++;
        targetheading = gyro.getHeading() - 135;
        gyro.resetZAxisIntegrator();
    }

    public void turnLeft45() throws InterruptedException {
        while (opModeIsActive() && (i5 == 1)) {
            if (gyro.getHeading() < 328 && gyro.getHeading() > 10) {
                leftWheel.setPower(0);
                rightWheel.setPower(0);
                i5--;
            } else {
                rightWheel.setPower(-.2);
            }
        }
        i5++;
        gyro.resetZAxisIntegrator();
    }

    public void turnRight45() throws InterruptedException {
        while (opModeIsActive() && (i6 == 1)) {
            if (gyro.getHeading() > 32 && gyro.getHeading() < 350) {
                leftWheel.setPower(0);
                rightWheel.setPower(0);
                i6--;
            } else {
                leftWheel.setPower(-.1);
            }
        }
        i6++;
        gyro.resetZAxisIntegrator();
    }

    public void backUp() throws InterruptedException {
        while (opModeIsActive() && (i4 == 1)) {
            if (frontRange.cmUltrasonic() < 25) {
                leftWheel.setPower(.15);
                rightWheel.setPower(.15);
            } else {
                leftWheel.setPower(0);
                rightWheel.setPower(0);
                i4--;
            }
        }
        i4++;
        gyro.resetZAxisIntegrator();
    }

    public void straightUntilWhite() throws InterruptedException {
        while (opModeIsActive() && (i10 == 1)) {
            if (floor.green() > 4) {
                i10--;
            } else {
                leftWheel.setPower(-.15);
                rightWheel.setPower(-.15);
            }
        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        i10++;
        gyro.resetZAxisIntegrator();
    }

    public void straightUntilWhite2() throws InterruptedException {
        leftWheel.setPower(-.25);
        rightWheel.setPower(-.25);
        sleep(200);
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        sleep(200);
        while (opModeIsActive() && (i10 == 1)) {
            if (floor.green() > 4) {
                i10--;
            } else {
                leftWheel.setPower(-.15);
                rightWheel.setPower(-.15);
            }
        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        i10++;
        gyro.resetZAxisIntegrator();
    }

    /*public void straightUntilWhite3() throws InterruptedException {
        while (opModeIsActive() && (i10 == 1)) {
            if (floor2.green() > 15) {
                i10--;
            } else {
                leftWheel.setPower(-.15);
                rightWheel.setPower(-.15);
            }
        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        i10++;
        gyro.resetZAxisIntegrator();
    }
    */

    public void blueBeacon() throws InterruptedException {
        if(beacon.blue() > beacon.red()) {

            buttonPusher.setPosition(1);

            sleep(500);

            buttonPusher.setPosition(.43);
        } else {
            buttonPusher.setPosition(0);

            sleep(500);

            buttonPusher.setPosition(.43);
        }
        gyro.resetZAxisIntegrator();
    }

    public void redBeacon() throws InterruptedException {
        if(beacon.red() > beacon.blue()) {
            buttonPusher.setPosition(1);

            sleep(1000);

            buttonPusher.setPosition(.43);
        } else {
            buttonPusher.setPosition(0);

            sleep(1000);

            buttonPusher.setPosition(.43);
        }
        gyro.resetZAxisIntegrator();
    }

    public void redBeacon2() throws InterruptedException {
        if(beacon.red() > 2) {
            buttonPusher.setPosition(1);

            sleep(3000);

            buttonPusher.setPosition(.43);
        } else {
            buttonPusher.setPosition(0);

            sleep(3000);

            buttonPusher.setPosition(.43);
        }
        gyro.resetZAxisIntegrator();
    }

    public void parkBeacon() throws InterruptedException {
        while(opModeIsActive() && frontRange.cmUltrasonic() > 10) {
            if(floor.green() < 4) {
                leftWheel.setPower(-.18);
                rightWheel.setPower(.06);
            } else {
                leftWheel.setPower(.06);
                rightWheel.setPower(-.18);
            }
        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

    public void parkBeacon2() throws InterruptedException {
        while(opModeIsActive() && frontRange.cmUltrasonic() > 10) {
            if(floor.green() < 4) {
                leftWheel.setPower(-.18);
                rightWheel.setPower(.06);
            } else {
                leftWheel.setPower(.06);
                rightWheel.setPower(-.18);
            }
        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

/*
    public void ImprovedParkBeacon() throws InterruptedException {
        while(opModeIsActive() && frontRange.cmUltrasonic() > 10) {
            if(floor.green() < 7 && floor2.green() < 7) {
                leftWheel.setPower(-.2);
                rightWheel.setPower(-.2);
            } else if(floor.green() < 7 && floor2.green() > 7) {
                leftWheel.setPower(-.2);
                rightWheel.setPower(-.1);
            } else if(floor.green() > 7 && floor2.green() < 7) {
                leftWheel.setPower(-.1);
                rightWheel.setPower(-.2);
            }
            leftWheel.setPower(0);
            rightWheel.setPower(0);
        }
    }
    */

    public void turnRedBeacon() throws InterruptedException {
        while(opModeIsActive() && i9 == 1) {
            if(leftODS.getLightDetected() < .07 && rightODS.getLightDetected() < .07 && rightODS.getLightDetected() < .9) {
                rightWheel.setPower(.1);
                leftWheel.setPower(-.1);
                telemetry.addData("Stage1", "Yam");
                telemetry.update();
            } else if(leftODS.getLightDetected() > .07 && rightODS.getLightDetected() < .07 && rightODS.getLightDetected() < .9) {
                rightWheel.setPower(0);
                leftWheel.setPower(-.1);
                telemetry.addData("Stage2", "Yam");
                telemetry.update();
            } else if(leftODS.getLightDetected() < .07 && rightODS.getLightDetected() > .07 && leftODS.getLightDetected() < .9) {
                rightWheel.setPower(.1);
                leftWheel.setPower(0);
                telemetry.addData("Stage3", "Yam");
                telemetry.update();
            } else if(leftODS.getLightDetected() > .07 && rightODS.getLightDetected() > .07 && leftODS.getLightDetected() < .9) {
                rightWheel.setPower(0);
                leftWheel.setPower(0);
                telemetry.addData("Stage4", "Yam");
                telemetry.update();
                i9--;
            }
        }
        i9++;
    }
}