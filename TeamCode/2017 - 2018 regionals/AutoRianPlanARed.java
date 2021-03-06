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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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
@Autonomous(name = "Rian_PlanA_Red", group = "Rian")
@Disabled
public class AutoRianPlanARed extends AutoRelic {


    protected BNO055IMU imuSensor = null;

    protected HardwareRian robot= null;

    protected int leftBackStamp;
    protected int leftFrontStamp;
    protected int rightBackStamp;
    protected int rightFrontStamp;

    protected double liftMotorHolderPower = 0.3;

    public AutoRianPlanARed () {

        teamColor = "red";
        rightColumnDistance = 2550;
        centerColumnDistance = 3200;
        leftColumnDistance = 3850;
    }

    @Override
    public void init() {
        robot = new HardwareRian();
        robot.init(hardwareMap);

        leftMotors = new DcMotor[2];
        leftMotors[0] = robot.motorLeftFrontWheel;
        leftMotors[1] = robot.motorLeftBackWheel;
        rightMotors = new DcMotor[2];
        rightMotors[0] = robot.motorRightFrontWheel;
        rightMotors[1] = robot.motorRightBackWheel;

        jewelArm = robot.jewelArm;
        jewelHitter = robot.jewelHitter;

        jewelSensor = robot.jewelSensor;
        jewelSensorDistance = robot.jewelSensorDistance;
        imuSensor = robot.imu;

        jewelKicker = new JewelKicker(jewelSensor,jewelArm,jewelHitter,telemetry);
        jewelKicker.init();
        jewelKicker.jewelHitterRestPosition = 0.44;
        jewelArmPos = jewelKicker.jewelArmActionPosition;
        jewelHitterPos = jewelKicker.jewelHitterRestPosition;

        navigation = new Navigation(telemetry);
        navigation.pidControlHeading.setKp(0.004);
        navigation.pidControlHeading.setKi(0.002);
        navigation.pidControlHeading.setKd(0.0000001);
        navigation.maxTurnDeltaPower = 0.4;
        navigation.convergeCountThreshold = 6;
        navigation.angleErrorTolerance = 2.1;

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

    }

    @Override
    public void loop() {
        telemetry.addData("State: " , state);
        switch (state) {
            case 0:

                // jewel handling
                state = jewelKicker.loop(0, 1, teamColor);

                // hitter arm to avoid jewel holes
                jewelKicker.jewelArmActionPosition = jewelArmPos + 0.1*rand.nextDouble()-0.1;
                jewelKicker.jewelHitterRestPosition = jewelHitterPos + 0.03*rand.nextDouble()-0.03;

                vuforia.identifyGlyphCrypto();

                getWheelLandmarks();

                break;
            case 1:

                //read vumark
                computeGlyphColumnDistance();

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
                    telemetry.addData("left", robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 3;
                }

                break;
            case 3:
                // move straight
                if ( 0== moveByDistance(move2GlyphBoxPower, cryptoBoxDistance - 20)) {
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
                    state = 5;

                }

                break;
            case 5:

                time = System.currentTimeMillis();

                if (time - timeStamp < 1300) {
                    moveAtPower(backupPower);
                } else {
                    moveAtPower(0.0);
                    state = 6;
                }

                break;
            case 6:

                if (0 == moveByDistance(-0.80, backupDistance - 100)) {

                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    // lower glyph bars
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);

                    state = 7;

                }

                break;
            case 7:
                if (0 == moveByDistance(0.6, pushDistance + 150)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);

                    // lower glyph bars
                    VortexUtils.moveMotorByEncoder(robot.liftMotor,0, liftMotorHolderPower);
                    state = 8;
                }

                break;
            case 8:
                // backup
                if (0 == moveByDistance(-0.8, backupDistance - 200)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 9;
                }
                break;
            case 9:
                // turn 180
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower, fCenterTurnAngle + centerGlyphAngleOffset, robot.axleDistance, leftMotors, rightMotors)) {
                    state = 10;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }

                VortexUtils.moveMotorByEncoder(robot.liftMotor, liftMoveMotorPosition2, liftMotorMovePower);

                //lift the glyph bar
                //VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorHolderPower);

                break;
            case 10:
                // move to center
                if (0 == moveByDistance(0.8, glyph2CenterDistance)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 11;
                }

                //lift the glyph bar
                //VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorHolderPower);

                break;
            case 11:
                // correct angle just in case it got knocked out the course
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle,fGlyphTurnAngle+fCenterTurnAngle+centerGlyphAngleOffset,leftMotors,rightMotors)) {
                    state = 12;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    timeStamp = System.currentTimeMillis();
                }

                break;
            case 12:
                // turn 180 degrees slowly
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower, fCenterTurnAngle+glyphOffAngle, robot.axleDistance, leftMotors, rightMotors)) {
                    state = 13;
                    getWheelLandmarks();
                    moveAtPower(0.2);
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, liftMoveMotorPosition2, liftMotorMovePower);
                    navigation.resetTurn(leftMotors, rightMotors);
                    timeStamp = System.currentTimeMillis();
                }

                break;
            case 13:
                // stop for half a second
                if (System.currentTimeMillis() - timeStamp > 150) {
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 14;
                }
                break;
            case 14:
                // move forward back to the glyph box
                if (0 == moveByDistance(0.8, glyph2CenterDistance+1100)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 15;
                }

                break;
            /*
            case 14:
                // wait 1 second
                if (System.currentTimeMillis() - timeStamp > 300) {
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 15;
                }
                break;
            case 15:
                // turn 180
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower, fCenterTurnAngle, robot.axleDistance, leftMotors, rightMotors)) {
                    state = 16;
                    getWheelLandmarks();
                    moveAtPower(0.2);
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, liftMoveMotorPosition2, liftMotorMovePower);
                    navigation.resetTurn(leftMotors, rightMotors);
                    timeStamp = System.currentTimeMillis();
                }
                break;
            case 16:
                // wait 1 second
                if (System.currentTimeMillis() - timeStamp > 300) {
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 17;
                }
                break;
            case 17:
                // move forward the rest of the distance
                if (0 == moveByDistance(0.8, glyph2CenterDistance)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 18;
                }
            case 18:
                // move back to glyph grid
                if (0 == waitByDistance(0.9, -backupDistance+700)) {
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 19;
                }
                break;
            case 19:
                // release glyph
                releaseGlyph();
                if (System.currentTimeMillis() - timeStamp > 1000) {
                    getWheelLandmarks();
                    state = 20;
                }
                break;
            */
            case 15:
                // backup
                if (0 == moveByDistance(-0.3, 200)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 18;
                }
                break;
            case 16:
                // turn 90 degrees
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower, -90, robot.axleDistance, leftMotors, rightMotors)) {
                    state = 17;
                    getWheelLandmarks();
                    moveAtPower(0.2);
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, liftMoveMotorPosition2, liftMotorMovePower);
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 17:
                // move left by 300
                if (0 == sideMoveByDistance(0.8, 2000)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 18;
                }
                break;
            default:
                // stop
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

    // positive power moves left
    public int sideMoveByDistance (double power, int d) {
        int distance = Math.abs(d);
        if (power == 0) {
            return 0; // zero power do nothing
        } else if (power < 0){
            distance = -distance;
        }
        if (robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp > -distance
                && robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp < distance) {
            sideMoveAtPower(sideMovePower);
        } else {
            sideMoveAtPower(0.0);
            return 0;
        }
        return 1;
    }

    public void collectGlyph () {
        robot.leftLiftWheel1.setPower(-1.0);
        robot.leftLiftWheel2.setPower(-1.0);
        robot.leftLiftWheel3.setPower(-1.0);
        robot.rightLiftWheel1.setPower(1.0);
        robot.rightLiftWheel2.setPower(1.0);
        robot.rightLiftWheel3.setPower(1.0);
        robot.beltServo.setPower(-1.0);
    }

    public void releaseGlyph () {
        robot.leftLiftWheel1.setPower(1.0);
        robot.leftLiftWheel2.setPower(1.0);
        robot.leftLiftWheel3.setPower(1.0);
        robot.rightLiftWheel1.setPower(-1.0);
        robot.rightLiftWheel2.setPower(-1.0);
        robot.rightLiftWheel3.setPower(-1.0);
    }

    public void stopGlyphWheels(){
        robot.leftLiftWheel1.setPower(0.0);
        robot.leftLiftWheel2.setPower(0.0);
        robot.leftLiftWheel3.setPower(0.0);
        robot.rightLiftWheel1.setPower(0.0);
        robot.rightLiftWheel2.setPower(0.0);
        robot.rightLiftWheel3.setPower(0.0);
    }

}
