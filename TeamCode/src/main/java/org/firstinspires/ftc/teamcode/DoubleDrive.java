package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;


@TeleOp(name="DoublesDrive", group="Linear Opmode")
public class DoubleDrive extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot r;
    private double intakeSwitchTime = 0;
    private double dumpSwitchTime = 0;
    private double pivotSwitchTime = 0;
    private double encoderOverideSwitchTime = 0;

    private boolean autoState = false;

    @Override
    public void runOpMode() {
        r = new Robot(hardwareMap, runtime, telemetry, this);
        r.initHardware();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();



        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                r.dropMarker();
            } else if(gamepad1.dpad_down){
                r.markerDrop.setPosition(0);
            }
            mecanumDrive();

            //nested ifs for intake software
            /*if(gamepad2.a && (runtime.time() - intakeSwitchTime) > 0.3) {
                intakeSwitchTime = runtime.time();
                if (!(r.intakeState == Robot.IntakeState.IN)) {
                    r.intakeState = Robot.IntakeState.IN;
                }else{
                    r.intakeState = Robot.IntakeState.STOP;
                }
            } else if(gamepad2.b && (runtime.time() - intakeSwitchTime) > 0.3){
                intakeSwitchTime = runtime.time();
                if (!(r.intakeState == Robot.IntakeState.OUT)) {
                    r.intakeState = Robot.IntakeState.OUT;
                }else{
                    r.intakeState = Robot.IntakeState.STOP;
                }
            }

            r.setIntakeState();*/

            if(gamepad2.a){
                r.intake.setPower(0.5);
            } else if(gamepad2.b){
                r.intake.setPower(-0.5);
            } else{
                r.intake.setPower(0);
            }

            if(gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.right_trigger > 0.05 || gamepad2.left_trigger > 0.05){
                autoState = false;
            } else if(gamepad2.x || gamepad2.y){
                autoState = true;
            }

            if(autoState){
                if(gamepad2.x){
                    r.pivotTarget = r.pivotVerticalPosition;
                    //r.pulleyTarget = r.pulleySilverPosition;
                } else if(gamepad2.y){
                    r.pivotTarget = r.pivotGoldPosition;
                    //r.pulleyTarget = r.pulleyGoldPosition;
                }
                r.setPivotState();
                //r.setPulleyState();

            } else{
                if(gamepad2.right_trigger > 0.05){
                    r.setPivotPower(0.3);
                } else if(gamepad2.left_trigger > 0.05){
                    r.setPivotPower(-0.3);
                }else{
                    r.intakePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    r.intakePivot.setPower(0);
                }

                /*if(gamepad2.right_bumper){
                    r.setPulleyPower(0.2);
                } else if(gamepad2.left_bumper){
                    r.setPulleyPower(-0.2);
                } else{
                    r.pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    r.pulley.setPower(0);
                }*/
            }

            if(gamepad1.x) {
                r.climb.setPower(1);
            }else if(gamepad1.y){
                r.climb.setPower(-1);
            }else {
                r.climb.setPower(0);
            }




            telemetry.addData("Right Trigger: ", gamepad1.right_trigger);
            telemetry.addData("Pivot: ", r.pivotTarget);
            telemetry.addData("Right Front: ", r.rightFront.getPower());
            telemetry.addData("Right Back: ", r.rightBack.getPower());
            telemetry.addData("Left Front: ", r.leftFront.getPower());
            telemetry.addData("Left Back: ", r.leftBack.getPower());
            telemetry.addData("Pivot Position: ", r.intakePivot.getCurrentPosition());
            telemetry.addData("Target Position: ", r.intakePivot.getTargetPosition());
            telemetry.addData("servo: ", r.markerDrop.getPosition());
            telemetry.update();
            idle();

        }
    }



    public void tankDrive(){
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;
        r.setDrivePower(rightPower, rightPower, leftPower, leftPower);
    }

    public void mecanumDrive(){
        double rightX = gamepad1.right_stick_x;
        double leftX = gamepad1.left_stick_x;
        double leftY = -gamepad1.left_stick_y;

        double robotSpeed = Math.pow(leftX, 2) + Math.pow(leftY, 2);
        robotSpeed = Math.sqrt(robotSpeed);

        double changeDirectionSpeed = -rightX;

        double desiredRobotAngle = Math.atan2(-leftX, leftY);

        double frontLeft = robotSpeed * Math.sin(-desiredRobotAngle + (Math.PI / 4)) - changeDirectionSpeed;
        double frontRight = robotSpeed * Math.cos(-desiredRobotAngle + (Math.PI / 4)) + changeDirectionSpeed;
        double backLeft = robotSpeed * Math.cos(-desiredRobotAngle + (Math.PI / 4)) - changeDirectionSpeed;
        double backRight = robotSpeed * Math.sin(-desiredRobotAngle + (Math.PI / 4)) + changeDirectionSpeed;

        r.setDrivePower(frontRight, backRight, frontLeft, backLeft);

    }
}
