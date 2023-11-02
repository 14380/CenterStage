package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class RotateTransferRightCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public RotateTransferRightCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        armSubsystem.TransferRightRotate();
    }

    @Override
    public boolean isFinished(){

        return true;
    }


}