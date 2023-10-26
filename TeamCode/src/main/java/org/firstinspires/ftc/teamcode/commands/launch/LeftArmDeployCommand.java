package org.firstinspires.ftc.teamcode.commands.launch;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

public class LeftArmDeployCommand extends CommandBase {

    private LauncherSubsystem launcherSubsystem;

    public LeftArmDeployCommand(LauncherSubsystem launch){
        launcherSubsystem = launch;
    }

    @Override
    public void initialize(){

        this.launcherSubsystem.LeftDeployArm();
    }

}