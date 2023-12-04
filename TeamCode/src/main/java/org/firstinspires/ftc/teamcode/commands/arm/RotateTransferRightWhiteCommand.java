package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class RotateTransferRightWhiteCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public RotateTransferRightWhiteCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        armSubsystem.TransferRightRotateWhite();
    }

    @Override
    public boolean isFinished(){

        return true;
    }


}