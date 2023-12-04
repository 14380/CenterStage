package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SinglePixelDropAutoCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    private ElapsedTime timer;

    public SinglePixelDropAutoCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();
        armSubsystem.DropPixel();
    }

    @Override
    public boolean isFinished(){

        if(timer.milliseconds() > 50){
            armSubsystem.LockPixel();
            return true;
        }
        return false;
    }
}
