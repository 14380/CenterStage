package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class RotateTransferCenterCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public RotateTransferCenterCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        armSubsystem.HomeTransferRotate();
    }

    @Override
    public boolean isFinished(){

        return true;
    }


}