package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by musta on 10/13/2018.
 */

public class Robot{


    public final static double COUNTS_PER_REV = 1120; //AndyMark 40:1 motor
    public final static double WHEEL_DIAMETER = 4; //Wheel diameter of 4 inches
    public final static double DRIVE_GEAR_REDUCTION = 0.75; //No gear reduction applied
    public final static double COUNTS_PER_INCH = (COUNTS_PER_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER * Math.PI);


    public GoldAlignDetector detector;

    public enum IntakeState {STOP, OUT, IN}

    public IntakeState intakeState;

    public int pivotRetractPosition = 0;
    public int pivotVerticalPosition = -1372;
    public int pivotGoldPosition = -1554;
    public int pivotMaxPosition = -2500;
    public int pivotTarget = pivotRetractPosition;

    public int pulleyExtendPosition = 2200;
    public int pulleyRetractPosition = 0;
    public int pulleyGoldPosition = 1780;
    public int pulleySilverPosition = 0;
    public int pulleyTarget = pulleyRetractPosition;



    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor intakePivot = null;
    public DcMotor pulley = null;
    public DcMotor climb = null;
    public DcMotor intake = null;
    public Servo markerDrop = null;

    public BNO055IMU imu = null;
    public Orientation angles = null;

    private HardwareMap hwMap = null;
    private LinearOpMode opMode = null;
    private ElapsedTime runtime = null;
    private Telemetry telemetry = null;

    public enum SamplingPositions {LEFT, RIGHT, CENTER}
    public SamplingPositions samplingPosition;

    public Robot(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry, LinearOpMode opMode){
        this.hwMap = hardwareMap;
        this.runtime = runtime;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    public void initHardware(){
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");


        intakePivot = hwMap.get(DcMotor.class, "intakePivot");
        pulley = hwMap.get(DcMotor.class, "pulley");
        climb = hwMap.get(DcMotor.class, "climb");
        markerDrop = hwMap.get(Servo.class, "markerDrop");
        intake = hwMap.get(DcMotor.class, "intake");

        pulley.setDirection(DcMotorSimple.Direction.REVERSE);
        intakePivot.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        climb.setDirection(DcMotorSimple.Direction.FORWARD);

        pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        markerDrop.setPosition(0);


    }

    public void initAuto() {
        initHardware();
        initIMU();
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

/*
        detector = new GoldAlignDetector();
        detector.init(hwMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 179; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();*/
    }

    public void initIMU(){
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled = true;
        IMUparameters.loggingTag = "IMU";
        IMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);
    }

    public void setDriveMode(DcMotor.RunMode mode){
        rightFront.setMode(mode);
        rightBack.setMode(mode);
        leftFront.setMode(mode);
        leftBack.setMode(mode);
    }

    public void setDrivePower(double rightFrontPower, double rightBackPower, double leftFrontPower, double leftBackPower){
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
    }

    public void setIntakeState(){
        if(intakeState == IntakeState.IN){
            intake.setPower(0.5);
        } else if(intakeState == IntakeState.OUT){
            intake.setPower(-0.5);
        } else{
            intake.setPower(0);
        }
    }

    public void setPivotState(){
        intakePivot.setTargetPosition(pivotTarget);
        intakePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakePivot.setPower(0.2);

        if(!intakePivot.isBusy()){
            intakePivot.setPower(0);
        }
    }
    public void setPulleyState(){
        pulley.setTargetPosition(pulleyTarget);
        pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulley.setPower(0.2);

        if(!pulley.isBusy()){
            pulley.setPower(0);
        }
    }

    public void setPivotPower(double power){
        intakePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(intakePivot.getCurrentPosition() < pivotMaxPosition && power < 0){
            intakePivot.setPower(power);
        } else if(intakePivot.getCurrentPosition() > pivotRetractPosition && power > 0){
            intakePivot.setPower(0);
        } else{
            intakePivot.setPower(power);
        }
    }

    public void setPulleyPower(double power){
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(pulley.getCurrentPosition() > pulleyExtendPosition && power < 0){
            pulley.setPower(0);
        } else if(pulley.getCurrentPosition() < pulleyRetractPosition && power > 0){
            pulley.setPower(0);
        } else{
            pulley.setPower(power);
        }
    }

    public void driveMotor(DcMotor motor, double distance, double power, double timeOut){
        int target = (int)(distance * COUNTS_PER_INCH);
        target = motor.getCurrentPosition() + target;
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        while(motor.isBusy()){
            opMode.idle();
        }

        motor.setPower(0);
    }


    public void encoderDrive(double leftDistance, double rightDistance, double leftPower, double rightPower, double timeOut){
        timeOut += runtime.time();

        int targetRight = (int)(rightDistance * COUNTS_PER_INCH);
        int targetLeft = (int)(leftDistance * COUNTS_PER_INCH);

        double rightFrontDistance = rightFront.getCurrentPosition() + targetRight;
        double rightBackDistance = rightBack.getCurrentPosition() + targetRight;
        double leftFrontDistance = leftFront.getCurrentPosition() + targetLeft;
        double leftBackDistance = leftBack.getCurrentPosition() + targetLeft;

        rightFront.setTargetPosition((int)rightFrontDistance);
        rightBack.setTargetPosition((int)rightBackDistance);
        leftFront.setTargetPosition((int)leftFrontDistance);
        leftBack.setTargetPosition((int)leftBackDistance);

        telemetry.addData("Start Position: ", leftBack.getCurrentPosition());
        telemetry.addData("Target Position: ", leftBack.getTargetPosition());
        telemetry.update();

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(rightPower, rightPower, leftPower, leftPower);


        while((rightFront.isBusy() && rightBack.isBusy() && leftFront.isBusy() && leftBack.isBusy()) &&
                timeOut > runtime.time() && opMode.opModeIsActive()){
            opMode.idle();
            telemetry.addData("State: ", "In Loop");
            telemetry.addData("Target Position: ", leftBack.getTargetPosition());
            telemetry.addData("Current Position: ",  leftBack.getCurrentPosition());
            telemetry.update();
        }

        resetTargets();
        setDrivePower(0, 0, 0, 0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void getSamplePosition(){
        if(getZAngle() > 10){
            samplingPosition = SamplingPositions.LEFT;
        } else if(getZAngle() < -20){
            samplingPosition = SamplingPositions.RIGHT;
        }
        else{
            samplingPosition = SamplingPositions.CENTER;
        }
    }

    public void getSamplePositionVision(){
        if(detector.isFound()){
            double blockPosition = detector.getXPosition();
            if(blockPosition < 300){
                samplingPosition = SamplingPositions.LEFT;
            } else{
                samplingPosition = SamplingPositions.CENTER;
            }
        } else{
            samplingPosition = SamplingPositions.RIGHT;
        }
    }

    public void turnToGold(){
        if(samplingPosition == SamplingPositions.LEFT){
            turnUntilAligned(0.2, -0.2);
        } else{
            turnUntilAligned(-0.2, 0.2);
            turnUntilAligned(-0.2, 0.2);
        }
    }

    public void timeDrive(double leftPower, double rightPower, double time){
        time += runtime.time();
        setDrivePower(rightPower, rightPower, leftPower, leftPower);
        while(time > runtime.time() && opMode.opModeIsActive()){
            opMode.idle();
        }
        setDrivePower(0, 0, 0, 0);
    }

    public void mecanumLeft(double distance, double power, double timeOut){
        timeOut += runtime.time();

        double rightFrontDistance = rightFront.getCurrentPosition() + (-distance*COUNTS_PER_INCH);
        double rightBackDistance = rightBack.getCurrentPosition() + (distance*COUNTS_PER_INCH);
        double leftFrontDistance = leftFront.getCurrentPosition() + (distance*COUNTS_PER_INCH);
        double leftBackDistance = rightBack.getCurrentPosition() + (-distance*COUNTS_PER_INCH);

        rightFront.setTargetPosition((int)rightFrontDistance);
        rightBack.setTargetPosition((int)rightBackDistance);
        leftFront.setTargetPosition((int)leftFrontDistance);
        leftBack.setTargetPosition((int)leftBackDistance);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power, power, power, power);

        while(rightFront.isBusy() && rightBack.isBusy() && leftFront.isBusy() && leftBack.isBusy() &&
                timeOut > runtime.time() && opMode.opModeIsActive()){
            opMode.idle();
        }
        resetTargets();
        setDrivePower(0, 0, 0, 0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void mecanumRight(double distance, double power, double timeOut){
        timeOut += runtime.time();

        double rightFrontDistance = rightFront.getCurrentPosition() + (distance*COUNTS_PER_INCH);
        double rightBackDistance = rightBack.getCurrentPosition() + (-distance*COUNTS_PER_INCH);
        double leftFrontDistance = leftFront.getCurrentPosition() + (-distance*COUNTS_PER_INCH);
        double leftBackDistance = rightBack.getCurrentPosition() + (distance*COUNTS_PER_INCH);

        rightFront.setTargetPosition((int)rightFrontDistance);
        rightBack.setTargetPosition((int)rightBackDistance);
        leftFront.setTargetPosition((int)leftFrontDistance);
        leftBack.setTargetPosition((int)leftBackDistance);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power, power, power, power);

        while(rightFront.isBusy() && rightBack.isBusy() && leftFront.isBusy() && leftBack.isBusy() &&
                timeOut > runtime.time() && opMode.opModeIsActive()){
            opMode.idle();
        }

        resetTargets();

        setDrivePower(0, 0, 0, 0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetTargets(){
        rightFront.setTargetPosition(rightFront.getCurrentPosition());
        rightBack.setTargetPosition(rightBack.getCurrentPosition());
        leftFront.setTargetPosition(leftFront.getCurrentPosition());
        leftBack.setTargetPosition(leftBack.getCurrentPosition());
    }



    public double getZAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return angles.secondAngle;
    }
    public void turnWithGyro(int degrees, double power, double timeOut, double buffer){
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startPosition = getZAngle();
        timeOut += runtime.time();

        while((Math.abs(startPosition - degrees) > buffer) && timeOut > runtime.time() && opMode.opModeIsActive()){
            if(startPosition > degrees){
                setDrivePower(-power, -power, power, power);
            }else if(startPosition < degrees){
                setDrivePower(power, power,  -power, -power);
            }
            opMode.idle();
            startPosition = getZAngle();
        }
        setDrivePower(0, 0, 0, 0);
    }





    public void descend(){
         double descendTime = 5.3;
        descendTime += runtime.time();

        climb.setPower(-1);
        while(descendTime > runtime.time() && opMode.opModeIsActive()){
            opMode.idle();
        }
        climb.setPower(0);
    }

    public void alignWithGold(double maximumDistance, double power, double timeOut, int buffer){
        timeOut += runtime.time();
        double blockPosition;
        if(detector.isFound()){
             blockPosition = detector.getXPosition();
        } else{
            blockPosition = 600;
        }

        double rightFrontDistance = rightFront.getCurrentPosition() + (maximumDistance*COUNTS_PER_INCH);
        double rightBackDistance = rightBack.getCurrentPosition() + (-maximumDistance*COUNTS_PER_INCH);
        double leftFrontDistance = leftFront.getCurrentPosition() + (-maximumDistance*COUNTS_PER_INCH);
        double leftBackDistance = rightBack.getCurrentPosition() + (maximumDistance*COUNTS_PER_INCH);

        rightFront.setTargetPosition((int)rightFrontDistance);
        rightBack.setTargetPosition((int)rightBackDistance);
        leftFront.setTargetPosition((int)leftFrontDistance);
        leftBack.setTargetPosition((int)leftBackDistance);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power, power, power, power);

        while((blockPosition-300 > buffer) && rightFront.isBusy() && rightBack.isBusy() && leftFront.isBusy() && leftBack.isBusy() &&
                timeOut > runtime.time() && opMode.opModeIsActive()){
            opMode.idle();
        }

        resetTargets();
        setDrivePower(0, 0, 0, 0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnUntilAligned(double rightPower, double leftPower){
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDrivePower(rightPower, rightPower, leftPower, leftPower);
        while(!detector.getAligned()){
            opMode.idle();
        }
        setDrivePower(0, 0, 0, 0);
    }

    public void unhook(){

    }


    public void turnTowardsDepot(){
        /*getSamplePosition();
        if(samplingPosition == SamplingPositions.RIGHT){
            turnWithGyro(-20, 0.2, 5, 3);
        } else if(samplingPosition == SamplingPositions.LEFT){
            turnWithGyro(20, 0.2, 5, 3);
        } else{
            turnWithGyro(0, 0.2, 5, 3);
        }*/
        if(samplingPosition == SamplingPositions.RIGHT){
            turnWithGyro(20, 0.2, 4, 3);
        }
        int angle = (int)getZAngle();
        turnWithGyro(-angle, 0.2, 5, 5);
    }

     public void turnWithGyroPivot(int degrees, double power, double timeOut, double buffer){
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startPosition = getZAngle();
        timeOut += runtime.time();

        while((Math.abs(startPosition - degrees) > buffer) && timeOut > runtime.time() && opMode.opModeIsActive()){
            if(startPosition > degrees){
                setDrivePower(0 ,0, power, power);
            }else if(startPosition < degrees){
                setDrivePower(power, power, 0, 0);
            }
            opMode.idle();
            startPosition = getZAngle();
        }
        setDrivePower(0, 0, 0, 0);
    }


    public void waitSeconds(double seconds){
        seconds += runtime.time();
        while(seconds>runtime.time() && opMode.opModeIsActive()){
            opMode.idle();
        }
    }

    public void dropMarker(){
        markerDrop.setPosition(0.6);
    }



}