package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmRightCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public ArmRightCommand(ArmSubsystem arm){
        armSubsystem = arm;

    }

    @Override
    public void initialize() {

        armSubsystem.ArmRightPosition();
    }

    @Override
    public boolean isFinished(){

        return this.armSubsystem.IsArmRight();
    }


}
