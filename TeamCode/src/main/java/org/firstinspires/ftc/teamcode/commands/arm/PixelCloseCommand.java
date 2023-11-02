package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class PixelCloseCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public PixelCloseCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        armSubsystem.LockPixel();
    }

    @Override
    public boolean isFinished(){

        return true;
    }
}