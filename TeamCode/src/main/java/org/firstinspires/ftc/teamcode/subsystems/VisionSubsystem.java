package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.CenterStageVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class VisionSubsystem extends SubsystemBase {

    private final CenterStageVisionProcessor processor;

    private VisionPortal visionPortal;

    private CenterStageVisionProcessor.StartingPosition startingPos;

    private Telemetry tele;

    public VisionSubsystem(HardwareMap map, Telemetry telemetry){

        tele = telemetry;
        processor = new CenterStageVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(map.get(WebcamName.class, "Webcam 1"), processor);
    }

    public CenterStageVisionProcessor.StartingPosition getPosition(){
        return startingPos;
    }

    public void StopStreaming(){
        visionPortal.stopStreaming();
    }

    @Override
    public void periodic(){
        startingPos = processor.getStartingPosition();
        tele.addData("Vision", startingPos);
        tele.update();
    }
}
