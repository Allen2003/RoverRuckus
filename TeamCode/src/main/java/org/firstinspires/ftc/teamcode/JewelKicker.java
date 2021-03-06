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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;
import java.util.Random;

class JewelKicker {

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     *
     */

    private ColorSensor jewelSensor;
    private Servo jewelArm;
    private Servo jewelHitter;

    private int state;
    private long timeStamp;
    double jewelArmRestPosition = 0.9;
    double jewelArmActionPosition = 0.15;
    double jewelHitterRestPosition =0.45;
    double jewelHitterRedPosition = 0.0;
    double jewelHitterBluePosition = 1.0;
    private Telemetry telemetry;

    long jewelWaitTime = 1000;
    long jewelBailOutTime = 5000;

    JewelKicker (ColorSensor c,
                 Servo arm,
                 Servo h,
                 Telemetry t) {
        jewelSensor = c;
        jewelArm = arm;
        jewelHitter = h;
        telemetry = t;
    }

    public void init() {
        jewelArm.setPosition(jewelArmRestPosition); // 0.8
        jewelHitter.setPosition(jewelHitterRestPosition); // 0.5
    }

    public void start() {
        state = 0;
        jewelHitter.setPosition(jewelHitterRestPosition);
        timeStamp = System.currentTimeMillis();
    }

    public int loop (int callerStartState, int callerEndState, String teamColor) {
        int returnState = callerStartState;
        switch (state) {
            case 0:
                telemetry.addData("Jewel arm pos ", jewelArmActionPosition);
                jewelArm.setPosition(jewelArmActionPosition);
                telemetry.addData("Jewel hitter pos ", jewelHitterRestPosition);
                jewelHitter.setPosition(jewelHitterRestPosition);
                if(System.currentTimeMillis() - timeStamp > jewelWaitTime) {
                    if (jewelSensor.blue() > jewelSensor.red()) {
                        if(teamColor == "blue") {
                            telemetry.addData("kick ", "blue");
                            jewelHitter.setPosition(jewelHitterRedPosition);
                        } else if (teamColor == "red") {
                            telemetry.addData("kick ", "red");
                            jewelHitter.setPosition(jewelHitterBluePosition);
                        }
                        timeStamp = System.currentTimeMillis();
                        state = 1;
                    } else if (jewelSensor.red() > jewelSensor.blue()) {
                        if(teamColor == "red") {
                            telemetry.addData("kick ", "red");
                            jewelHitter.setPosition(jewelHitterRedPosition);
                        } else if (teamColor == "blue") {
                            telemetry.addData("kick ", "blue");
                            jewelHitter.setPosition(jewelHitterBluePosition);
                        }
                        timeStamp = System.currentTimeMillis();
                        state = 1;
                    }
                }
                if(System.currentTimeMillis() - timeStamp > jewelBailOutTime) {
                    state = 1; // give up
                }
                telemetry.addData("Red  ", jewelSensor.red());
                telemetry.addData("Green", jewelSensor.green());
                telemetry.addData("Blue ", jewelSensor.blue());
                telemetry.addData("ArmPosition", jewelArm.getPosition());
                telemetry.addData("HitterPosition", jewelHitter.getPosition());
                telemetry.addData("State", state);
                break;
            case 1:
                if(System.currentTimeMillis() - timeStamp > 500) {
                    state = 2;
                    timeStamp = System.currentTimeMillis();
                }
                break;
            case 2:
                jewelArm.setPosition(jewelArmRestPosition);
                if(System.currentTimeMillis() - timeStamp > 500) {
                    jewelHitter.setPosition(0.50);
                    state = 3;
                }
                break;
            default:
                returnState = callerEndState;
                break;
        }

        telemetry.update();
        return returnState;
    }
}
