package org.firstinspires.ftc.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by wdhoward on 12/2/16.
 */
@Autonomous(name = "MiniBot: Play recent ENCODER", group = "MINI DOPPLE")
//@Disabled

public class MiniDoppleBotPlaybackRecentWithEncoderDrive extends MiniDoppleBotPlaybackRecent {

    private String LOG_TAG = "MINIBOT PLAYBACK WITH ENCODER - ";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.encoderDriveIsEnabled = true;
        super.runOpMode();
    }
}
