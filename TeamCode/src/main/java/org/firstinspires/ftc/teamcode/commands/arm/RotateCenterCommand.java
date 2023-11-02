package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class RotateCenterCommand  extends CommandBase {

    private ArmSubsystem armSubsystem;

    public RotateCenterCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        armSubsystem.ArmCenterPosition();
    }

    @Override
    public boolean isFinished(){

        return true;
    }


}