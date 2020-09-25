package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.robots.beachcomber.PoseSkystone;

public class UserCommand {
    static volatile char newCommand;


    public void Set(char command){newCommand=command;}

    public void Update(PoseSkystone pose){
        switch (newCommand){
            case 'a': //align IMU to GPS
                newCommand = ' ';
                pose.articulate(PoseSkystone.Articulation.alignIMUtoGPS);
                break;
            case 'd': //set dpgr path
                newCommand = ' ';
                break;
            case '6': //6832 path
                newCommand = ' ';
                break;
            case 'b': //blue square test path
                newCommand = ' ';
                break;
            default: //return to manual drive
                newCommand = ' ';
                break;
        }

    }
}
