package org.firstinspires.ftc.teamcode.commands.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class StopStreamingCommand extends CommandBase {

    private VisionSubsystem visionSubsystem;

    public StopStreamingCommand(VisionSubsystem vision){
        this.visionSubsystem = vision;
    }

    @Override
    public void initialize(){

        this.visionSubsystem.StopStreaming();
    }
    @Override
    public boolean isFinished(){

        return true;
    }

}
