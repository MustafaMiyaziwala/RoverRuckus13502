/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Double Sampling", group="Linear Opmode")

public class FullBlueAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot r;
    public GoldAlignDetector detector;

    @Override
    public void runOpMode(){
        r = new Robot(hardwareMap, runtime, telemetry, this);
        r.initAuto();
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
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

        detector.enable();
        r.descend();
        getSamplePositionVision();
        r.mecanumLeft(4, 0.2, 5);
        r.encoderDrive(8, 8, 0.4, 0.4, 5);
        turnToGold();
        detector.disable();
        r.encoderDrive(30, 30, 0.6, 0.6, 6);

       if(r.samplingPosition == Robot.SamplingPositions.CENTER){
            r.encoderDrive(24, 24, 0.5, 0.5, 5);
        } else if(r.samplingPosition == Robot.SamplingPositions.RIGHT){
            r.encoderDrive(15.5, 15.5, 0.5, 0.5, 5);
        } else{
            r.encoderDrive(10, 10, 0.6, 0.6, 5);
            r.turnWithGyro(-45, 0.3, 5, 7);
            r.encoderDrive(32, 32, 0.6, 0.6, 5);

        }
        r.turnWithGyro(-123, 0.3, 6, 8);
       // r.mecanumLeft(4, 0.3, 5);
        if(r.samplingPosition == Robot.SamplingPositions.RIGHT){
            r.encoderDrive(-20,-20,0.7,0.7, 5);
        }

        r.dropMarker();
        r.waitSeconds(0.5);
        if(r.samplingPosition == Robot.SamplingPositions.RIGHT){
            r.encoderDrive(43,43,0.7,0.7,5);
        } else if(r.samplingPosition == Robot.SamplingPositions.CENTER){
            r.encoderDrive(45,45,0.7,0.7,5);
        } else{
            r.encoderDrive(49,49,0.7,0.7,5);
        }

        r.turnWithGyro(-172, 0.5, 5, 4);

        if(r.samplingPosition == Robot.SamplingPositions.LEFT){
            r.encoderDrive(29, 29, 0.6, 0.6, 5);

        } else if (r.samplingPosition == Robot.SamplingPositions.CENTER){
            r.encoderDrive(40,40,0.6,0.6,5);
        } else{
            r.encoderDrive(63, 63, 0.6, 0.6, 5);
        }

        if(r.samplingPosition == Robot.SamplingPositions.CENTER){
            r.turnWithGyro(-90, 0.4, 5, 5);
            r.encoderDrive(5, 5, 0.3, 0.3, 5);
        } else if (r.samplingPosition == Robot.SamplingPositions.RIGHT){
            r.turnWithGyro(-90, 0.2, 5, 4);
            r.encoderDrive(5, 5, 0.3, 0.3, 5);
        } else{
            r.turnWithGyro(-90, 0.2, 5, 4);
            r.encoderDrive(7, 7, 0.3, 0.3, 5);
        }


        /*r.turnTowardsDepot();
        if(r.samplingPosition == Robot.SamplingPositions.RIGHT){
            r.encoderDrive(8, 5, 0.2, 0.2, 5);`
        } else{
            r.encoderDrive(20, 20, 0.2, 0.2, 5);
        }

        r.turnWithGyro(-140, 0.2, 6, 8);

        if(r.samplingPosition == Robot.SamplingPositions.LEFT){
            r.mecanumLeft(17, 0.3, 5);
            r.dropMarker();
        } else if(r.samplingPosition == Robot.SamplingPositions.CENTER){
            r.mecanumLeft(9, 0.3, 5);
            r.dropMarker();
        } else{
            r.mecanumLeft(2, 0.3, 5);
            r.encoderDrive(-10, -10, 0.2, 0.2, 4);
            r.dropMarker();
        }



        r.encoderDrive(60, 60, 0.4, 0.4, 5);
        */
       //r.mecanumLeft(10, 0.4, 5);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            idle();
            // Show the elapsed game time and wheel power.

        }


    }

    public void getSamplePositionVision(){
        if(detector.isFound()){
            double blockPosition = detector.getXPosition();
            if(blockPosition < 300){
                r.samplingPosition = Robot.SamplingPositions.LEFT;
            } else{
                r.samplingPosition = Robot.SamplingPositions.CENTER;
            }
        } else{
            r.samplingPosition = Robot.SamplingPositions.RIGHT;
        }
    }

    public void turnUntilAligned(double rightPower, double leftPower){
        r.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.setDrivePower(rightPower, rightPower, leftPower, leftPower);
        while(!detector.getAligned()){
            idle();
        }
        r.setDrivePower(0, 0, 0, 0);
    }

    public void turnToGold(){
        if(r.samplingPosition == Robot.SamplingPositions.LEFT){
            turnUntilAligned(0.2, -0.2);
        } else{
            turnUntilAligned(-0.2, 0.2);
            turnUntilAligned(-0.2, 0.2);
        }
    }
}

