package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AutonomousFramework", group = "test")
public class AutonomousFramework extends LinearOpMode {

    int state = 1;
    boolean A = false;
    boolean B = false;
    OpenCvCamera webcam;



    @Override
    public void runOpMode() throws InterruptedException {

        //Roadrunner initialization
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        VisionPipeline pipeline = new VisionPipeline();
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        //opens connection to camera asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case 1: // Grab wobble goal
                    break;

                case 2: // Drive forward to rings
                    break;

                case 3: // Detect number of rings, determine A, B, C

                case 4: // Straighten robot
                    if (B) {
                        goToNextState(); // path B
                    } else {
                        goToState(7); // path A/C
                    }
                    break;

                case 5: // Path B: Go straight forward until 2 lines have been detected
                    break;

                case 6: // Path B: Go forward 12 inches
                    goToState(11); // drop wobble goal
                    break;

                case 7: // Path A/C: Strafe towards wall
                    if (A) {
                        goToNextState(); // path A
                    } else {
                        goToState(9); // path C
                    }
                break;

                case 8: // Path A: Go straight forward until 2 lines have been detected
                    goToState(11); // drop wobble goal
                    break;

                case 9: // Path C: Go straight forward until 4 lines have been detected
                    break;

                case 10: // Path C: Go forward 12 inches
                    break;

                case 11: // Release wobble goal
                    break;

                case 12: // Back up behind white line
                    break;

                case 13: // Strafe towards center until 36 inches away from wall
                    break;

                case 14: // Launch rings
                    break;

                case 15: // Park on white line
                    break;

            }

            telemetry.addData("Cb Value", pipeline.getAnalysis());
            telemetry.addData("Ring Position", pipeline.position);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

        }
    }

    public void goToNextState() { state++; }
    public void goToState(int newState) { state = newState; }
}
