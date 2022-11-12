/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="SleeveColorDetectorRIGHT", group="Linear Opmode")
public class SleeveColorDetectorTestRIGHT extends LinearOpMode
{
    SleeveColorDetectorDoubleWebcam colorDetector;

    @Override
    public void runOpMode()
    {
        int cbValue = 0;
        int path = 3;
        String color = "Not Found";

//        colorDetector = new SleeveColorDetector(this);
//        colorDetector.initSleeveColorDetector();

        colorDetector = new SleeveColorDetectorDoubleWebcam(this,"RIGHT");

      colorDetector.InitSleeveColorDetectorDoubleWebcam();

        waitForStart();

        while(opModeIsActive()) {
            cbValue = colorDetector.getAnalysis();
            if(cbValue <=100) {
                color = "ORANGE";
                path = 1;
            }
            else if(cbValue >=140) {
                color = "PURPLE";
                path = 2;
            }
            else {
                color = "GREEN";
                path = 3;
            }
            telemetry.addData("Analysis: ", cbValue);
            telemetry.addData("The color is:", color);
            telemetry.update();
        }
    }

}
