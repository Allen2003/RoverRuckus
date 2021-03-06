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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "VLSB_PlanA_Red", group = "B_VLSB")
@Disabled
public class AutoVLSBPlanARed extends AutoRelic {


    protected BNO055IMU imuSensor = null;

    protected HardwareVLSB robot= null;

    protected int leftBackStamp;
    protected int leftFrontStamp;
    protected int rightBackStamp;
    protected int rightFrontStamp;


    public AutoVLSBPlanARed() {
        teamColor = "red";

        glyphLiftPosition = 1200;
        centerGlyphAngleOffset = 0;
        cryptoBoxDistance = -200;
        vuforiaDetectingPower = -0.4;
        move2GlyphBoxPower = -0.4;
        move2CenterPower = 0.8;
        fGlyphTurnAngle = -90;
        center2GlyphBoxPower = -0.8;
        glyTurnPower = -0.4;

        cryptoBoxDistance = 800;
        center2GlyphDistance = 3500;

        leftColumnDistance = 3860;
        centerColumnDistance = 3150;
        rightColumnDistance = 2500;
    }

    @Override
    public void init() {
        robot = new HardwareVLSB();
        robot.init(hardwareMap);

        leftMotors = new DcMotor[2];
        leftMotors[0] = robot.motorLeftFrontWheel;
        leftMotors[1] = robot.motorLeftBackWheel;
        rightMotors = new DcMotor[2];
        rightMotors[0] = robot.motorRightFrontWheel;
        rightMotors[1] = robot.motorRightBackWheel;
        robot.defaultGlyphWheelPower = 0.15;

        jewelArm = robot.jewelArm;
        jewelHitter = robot.jewelHitter;

        smolL = robot.smolL;

        jewelSensor = robot.jewelSensor;
        jewelSensorDistance = robot.jewelSensorDistance;
        imuSensor = robot.imu;

        jewelKicker = new JewelKicker(jewelSensor,jewelArm,jewelHitter,telemetry);
        //jewelKicker.init();
        jewelKicker.jewelArmActionPosition = 0.10;
        jewelKicker.jewelArmRestPosition = 0.55;

        jewelArmPos = jewelKicker.jewelArmActionPosition;
        jewelHitterPos = jewelKicker.jewelHitterRestPosition;

        navigation = new Navigation(telemetry);

        vuforia = new HardwareVuforia(VuforiaLocalizer.CameraDirection.BACK);
        vuforia.init(hardwareMap);

        telemetry.addData("jewelArm", jewelArm.getPosition());
        telemetry.addData("jewelHitter", jewelHitter.getPosition());
        telemetry.update();
    }


    @Override
    public void start() {
        robot.start();
        vuforia.start();
        state = 0;
        timeStamp = System.currentTimeMillis();
        vuforia.vumarkImage = "Unknown";
        jewelKicker.start();
        smolL.setPosition(0.55);
        robot.initAllDevices();

    }

    @Override
    public void loop() {
        switch (state) {
            case 0:

                // jewel handling
                state = jewelKicker.loop(0, 1, teamColor);

                // move jewel arm to avoid jewel holes
                jewelKicker.jewelArmActionPosition = jewelArmPos + 0.1*rand.nextDouble()-0.1;
                jewelKicker.jewelHitterRestPosition = jewelHitterPos + 0.03*rand.nextDouble()-0.03;

                vuforia.identifyGlyphCrypto();

                getWheelLandmarks();

                break;
            case 1:

                //read vumark
                computeGlyphColumnDistance();

                // lift glyph bar
                VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorHolderPower);

                //set jewel hitter position
                robot.jewelHitter.setPosition(1.00);
                robot.beltDepositGlyph();

                //move forward with encoder
                if (0 == moveByDistance(vuforiaDetectingPower, columnDistance)) {
                    vuforia.relicTrackables.deactivate();
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 2;
                }

                break;
            case 2:
                if (fGlyphTurnAngle == 0.0f || 0 == navigation.turnByEncoderOpenLoop(glyTurnPower,fGlyphTurnAngle,
                        robot.axleDistance, leftMotors, rightMotors)) {
                    turnAtPower(0.0);
                    telemetry.addData("left", robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp + robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp);
                    telemetry.addData("right", robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 3;
                }

                break;
            case 3:
                // move straight
                if (0 == moveByDistance(move2GlyphBoxPower, cryptoBoxDistance)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    state = 4;
                }

                break;
            case 4:
                // release the glyph

                time = System.currentTimeMillis();

                if (time - timeStamp < 1000) {
                    releaseGlyph();
                } else {

                    timeStamp = System.currentTimeMillis();
                    state = 6;

                }

                break;

            case 6:
                // back up from the crypto box
                if (0 == moveByDistance(-move2GlyphBoxPower, backupDistance)) {

                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();

                    // lower glyph bars
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);

                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 7;

                }
                break;
            case 7:
                // push glyph into place
                if (0 == moveByDistance(move2GlyphBoxPower*2, 700)) {

                    moveAtSpeed(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();

                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 8;

                }
                break;
            case 8:
                // correct angle just in case it got knocked out the cource
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle,fGlyphTurnAngle,leftMotors,rightMotors)) {
                    state = 9;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 9:
                // move to center
                if (0 == moveByDistance(move2CenterPower, glyph2CenterDistance/2 + backupDistance + 500)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    robot.beltDepositGlyph();
                    collectGlyph();
                    state = 10;
                }

                break;
            case 10:
                // move to center
                if (0 == moveByDistance(collectingGlyphPower, glyph2CenterDistance/3)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    robot.beltDepositGlyph();
                    state = 11;
                }

                break;
            case 11:
                if (System.currentTimeMillis() - timeStamp > 500) {
                    state = 12;
                    getWheelLandmarks();
                    stopGlyphWheels();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 12:
                // correct angle just increase it got knocked out of course
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle,fGlyphTurnAngle,leftMotors,rightMotors)) {
                    state = 13;
                    getWheelLandmarks();

                    // spit out jammed glyphs
                    robot.defaultGlyphWheelPower = 0.3;
                    //robot.glyphWheelUnload();

                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 13:
                // back up
                if (0 == moveByDistance(center2GlyphBoxPower, center2GlyphDistance/2)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 14;
                }

                break;
            case 14:
                //turn 180 degrees
                if(0 == navigation.turnByEncoderOpenLoop(glyTurnPower,fCenterTurnAngle+glyphOffAngle, robot.axleDistance, leftMotors, rightMotors)) {
                    state = 15;
                    getWheelLandmarks();
                    moveAtPower(0.2);
                    reverseBelt();
                    navigation.resetTurn(leftMotors, rightMotors);
                    timeStamp = System.currentTimeMillis();
                }

                break;
            case 15:
                // go forward
                if (0 == moveByDistance(-center2GlyphBoxPower, center2GlyphDistance/2 + 500)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 16;
                }

                break;
            case 16:
                // release glyph
                reverseWheels();
                if (System.currentTimeMillis() - timeStamp > 2000) {
                    getWheelLandmarks();
                    state = 17;
                }
                break;
            case 17:
                // backup
                if (0 == moveByDistance(move2GlyphBoxPower * 2, 300)) {
                    moveAtPower(0.0);
                    state = 18;
                }
                break;
            default:
                // stop
                vuforia.relicTrackables.deactivate();
                robot.stop();
                break;
        }

        telemetry.addData("state", state);
        telemetry.addData("vumark", vuforia.vumarkImage);
        telemetry.addData("teamColor", teamColor);
        telemetry.update();
    }

    public void getWheelLandmarks () {
        leftBackStamp = robot.motorLeftBackWheel.getCurrentPosition();
        leftFrontStamp = robot.motorLeftFrontWheel.getCurrentPosition();
        rightBackStamp = robot.motorRightBackWheel.getCurrentPosition();
        rightFrontStamp = robot.motorRightFrontWheel.getCurrentPosition();
        wheelDistanceLandMark = (leftBackStamp+leftFrontStamp+rightBackStamp+rightFrontStamp)/4;
    }

    public void sideMoveAtPower(double p) {
        robot.motorLeftFrontWheel.setPower(-p);
        robot.motorRightBackWheel.setPower(-p);
        robot.motorRightFrontWheel.setPower(p);
        robot.motorLeftBackWheel.setPower(p);
    }

    public void moveAtSpeed(double p){
        robot.motorLeftFrontWheel.setPower(p);
        robot.motorRightBackWheel.setPower(p);
        robot.motorRightFrontWheel.setPower(p);
        robot.motorLeftBackWheel.setPower(p);
    }

    // positive power moves left
    public int sideMoveByDistance (double power, int d) {
        int distance = Math.abs(d);
        if (power == 0) {
            return 0; // zero power do nothing
        }
        if (Math.abs(robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp) < distance
                && Math.abs(robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp) < distance) {
            sideMoveAtPower(sideMovePower);
        } else {
            sideMoveAtPower(0.0);
            return 0;
        }
        return 1;
    }

    public void releaseGlyph() {
        robot.smolL.setPosition(0.00);
        robot.beltDepositGlyph();
    }

    public void collectGlyph() {
        robot.smolL.setPosition(robot.blockerLoadPosition);
        robot.glyphPusher.setPosition(robot.pusherLoadPosition);
        robot.glyphWheelLoad();
        robot.beltDepositGlyph();
    }

    public void reverseBelt() {
        robot.beltSpitOutGlyph();
    }

    public void reverseWheels() {
        robot.smolL.setPosition(robot.blockerLoadPosition);
        robot.beltSpitOutGlyph();
        robot.glyphWheelUnload();
    }

    public void stopGlyphWheels(){
        robot.leftLiftWheel.setPower(0.0);
        robot.rightLiftWheel.setPower(0.0);
        robot.beltStop();
    }

}
