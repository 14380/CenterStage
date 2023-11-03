package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LauncherSubsystem extends SubsystemBase {

    private ServoImplEx planeLauncherServo;

    public LauncherSubsystem(HardwareMap map, RobotStateSubsystem state){

        planeLauncherServo = map.get(ServoImplEx.class, "planeLauncher");
        planeLauncherServo.setDirection(Servo.Direction.REVERSE);
        planeLauncherServo.setPosition(0);
    }

    public void LaunchPlane(){

        planeLauncherServo.setPosition(1);
    }



}
