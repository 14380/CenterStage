package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class DropPixelCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public DropPixelCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        armSubsystem.DropPixel();
    }

    @Override
    public boolean isFinished(){

        return true;
    }
}
