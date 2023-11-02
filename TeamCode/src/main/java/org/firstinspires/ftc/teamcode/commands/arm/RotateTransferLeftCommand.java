package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class RotateTransferLeftCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public RotateTransferLeftCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        armSubsystem.TransferLeftRotate();
    }

    @Override
    public boolean isFinished(){

        return true;
    }


}