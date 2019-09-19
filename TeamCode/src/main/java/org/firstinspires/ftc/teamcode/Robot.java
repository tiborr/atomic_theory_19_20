package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.blackbox.LogContext;
import org.firstinspires.ftc.teamcode.blackbox.LogSession;
import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.blackbox.MatchType;
import org.firstinspires.ftc.teamcode.blackbox.sensors.SensorFactory;
import org.firstinspires.ftc.teamcode.blackbox.sensors.WrappedBNO055IMU;
import org.firstinspires.ftc.teamcode.blackbox.sensors.WrappedLynxModule;
//import org.firstinspires.ftc.teamcode.blackbox.sensors.WrappedMuxedRangeSensor;
import org.firstinspires.ftc.teamcode.blackbox.sensors.WrappedTFObjectDetector;
import org.firstinspires.ftc.teamcode.blackbox.sensors.WrappedMRRangeSensor;
import org.firstinspires.ftc.teamcode.utils.LynxPumperRunnable;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;
import org.json.JSONException;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;
import java.util.List;

public class Robot implements AutoCloseable {
    /*
     * constants
     */
    public static final int MOTOR_PORT_FRONT_LEFT = 0;
    public static final int MOTOR_PORT_FRONT_RIGHT = 1;
    public static final int MOTOR_PORT_BACK_LEFT = 2;
    public static final int MOTOR_PORT_BACK_RIGHT = 3;

    /*
     * expansion hubs
     */

    public WrappedLynxModule expansionHub1;
    public WrappedLynxModule expansionHub2;

    /*
     * motors
     */

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    /*
     * sensors
     */

    public WrappedBNO055IMU imu;

    /*
     * state
     */

    public final LogSession logSession;

    private RobotFeature[] _features;

    public DcMotor.ZeroPowerBehavior driveMotorZeroPowerBehavior;
    public VuforiaLocalizer vuforia;
    public WrappedTFObjectDetector tfod;
    public LinearOpMode opMode;

    public Robot(MatchPhase phase, LinearOpMode opModeIn, RobotFeature[] features) throws InterruptedException {
        opMode = opModeIn;
        _features = features;

        /*
         * expansion hub initialization
         */
        expansionHub1 = SensorFactory.getSensor(opMode.hardwareMap, LynxModule.class, "expansion hub 1", "Expansion Hub 1");
        expansionHub2 = SensorFactory.getSensor(opMode.hardwareMap, LynxModule.class, "expansion hub 2", "Expansion Hub 2");

        /*
         * motor initialization
         */
        frontLeft = opMode.hardwareMap.dcMotor.get("front_left");
        frontRight = opMode.hardwareMap.dcMotor.get("front_right");
        backLeft = opMode.hardwareMap.dcMotor.get("back_left");
        backRight = opMode.hardwareMap.dcMotor.get("back_right");

        /*
         * motor setup
         */
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
         * sensor - imu
         */
        imu = SensorFactory.getSensor(opMode.hardwareMap, BNO055IMU.class, "imu", "imu");

        while (!opMode.isStopRequested() && !imu.isGyroCalibrated()) {
            Thread.sleep(50);
            opMode.telemetry.addLine("calibrating");
            opMode.telemetry.update();
            opMode.idle();
        }

        resetHeading();

        /*
         * initialize sensors
         */
//        if (isFeatureRequested(RobotFeature.CAMERA)) {
//            VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();
//            vuforiaParameters.vuforiaLicenseKey = Consts.VUFORIA_KEY;
//            vuforiaParameters.cameraName = leftWebcam;
//            vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);
//
//            int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
//            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//            TFObjectDetector tfodRaw = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//            tfod = new WrappedTFObjectDetector(tfodRaw, "tfod");
//            tfod.loadModelFromAsset(Consts.TFOD_MODEL_FILE, Consts.TFOD_LABEL_GOLD, Consts.TFOD_LABEL_SILVER);
//        }

        /*
         * set up logging
         */
        try {
            logSession = new LogSession(opMode, phase, MatchType.OTHER, "");
        } catch (IOException e) {
            // TODO: this is dumb
            throw new RuntimeException(e);
        }

        // TODO: clean me up
        String opModeName = "";
        if (opModeIn.getClass().getAnnotation(Autonomous.class) != null) {
            opModeName = opModeIn.getClass().getAnnotation(Autonomous.class).name();
        } else {
            opModeName = opModeIn.getClass().getAnnotation(TeleOp.class).name();
        }
        logSession.setFact("Name", opModeName);

        logSession.attachDatastreamable(expansionHub1);
        logSession.attachDatastreamable(expansionHub2);
        logSession.attachDatastreamable(imu);

        if (tfod != null) {
            logSession.attachDatastreamable(tfod);
        }
    }

    /*
     * feature functions
     */
    public boolean isFeatureRequested(RobotFeature featureToCheck) {
        for (RobotFeature feature : _features) {
            if (featureToCheck == feature) {
                return true;
            }
        }
        return false;
    }

    /*
     * logging functions
     */
    public void handleMatchStart() {
        logSession.startMatch();

//        Thread lynxPumper = new Thread(new LynxPumperRunnable(opMode, new WrappedLynxModule[] {
//                expansionHub1, expansionHub2
//        }));
//        lynxPumper.start();
    }

    @Override
    public void close() {
        try {
            logSession.close();
        } catch (IOException | JSONException e) {
            e.printStackTrace();
        }
    }

    /*
     * drive functions
     */
    public void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior newBehavior) {
        driveMotorZeroPowerBehavior = newBehavior;

        frontLeft.setZeroPowerBehavior(newBehavior);
        frontRight.setZeroPowerBehavior(newBehavior);
        backLeft.setZeroPowerBehavior(newBehavior);
        backRight.setZeroPowerBehavior(newBehavior);
    }

    public void setClippedMotorPower(DcMotor motor, double power) {
        if (Math.abs(power) < 0.2) {
            motor.setPower(0);
        } else {
            motor.setPower((power < 0 ? -1 : 1) * Math.min(Math.abs(power), 1));
        }
    }

    public void driveMotors(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void driveMotorsClipped(double fl, double fr, double bl, double br) {
        setClippedMotorPower(frontLeft, fl);
        setClippedMotorPower(frontRight, fr);
        setClippedMotorPower(backLeft, bl);
        setClippedMotorPower(backRight, br);
    }

    /*
     * sensor functions - imu
     */
    public double getHeading() {
        return imu.getHeading();
    }

    public void resetHeading() {
        imu.resetHeading();
    }

    /*
     * sensor functions - tensorflow
     */
    public void activateTfod() {
        tfod.activate();
    }

    public void deactivateTfod() {
        tfod.shutdown();
    }
}
