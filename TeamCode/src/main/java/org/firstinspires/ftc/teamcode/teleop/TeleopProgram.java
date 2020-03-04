package org.firstinspires.ftc.teamcode.teleop;

import android.media.MediaPlayer;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.util.annotations.MainRobot;
import com.SCHSRobotics.HAL9001.util.misc.BeatBox;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.misc.Toggle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robots.Cygnus;

/**
 * The teleop program for the 2019-2020 season.
 *
 * @author Cole Savage
 * @author Isabella Valdez Palmer
 * @author Jack Kinney
 * @author Tristan Tran
 * @author Jared Smith
 * @author Ella Player
 */
@TeleOp(name = "Teleop", group = "competition")
public class TeleopProgram extends BaseTeleop {
    //The name of the song cycle button.
    private static final String SONG_BUTTON = "soundfxButton";
    //The robot to be used in this program.
    public @MainRobot Cygnus robot;
    //The customizable gamepad used to control the sounds.
    private CustomizableGamepad gamepad;
    //The beatbox object used to contain and play the sounds.
    private BeatBox beatBox;
    //The toggle used to change the behavior of the song change button.
    private Toggle songToggle;
    //The index of the current song.
    private int songNum;
    //The name of the current song.
    private String currentSong;
    //The number of songs possible to play on the robot.
    private final static int NUMBER_OF_SONGS = 15;

    @Override
    protected void onInit() {
        //Creates the customizable gamepad and adds the song button.
        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(SONG_BUTTON,new Button(2, Button.BooleanInputs.x));
        //Starts the song index at 0.
        songNum = 0;
        //Creates the song toggle.
        songToggle = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false);
        //Creates the beatbox.
        beatBox = new BeatBox();

        //Adds all the songs to the beatbox.
        beatBox.addSong("Spooky Scary Skeletons (Remix)",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.spookyskeleboys));
        beatBox.addSong("Dual of Fates",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.dualoffates));
        beatBox.addSong("Ra Ra Rasputin",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.rararasputin));
        beatBox.addSong("Gaston",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.gastondos));
        beatBox.addSong("Giorno's Theme",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.ggthemebest));
        beatBox.addSong("Inferno - FIREFORCE OP",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.inferno));
        beatBox.addSong("Istanbul Not Constantinople",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.isanbul));
        beatBox.addSong("Megalovannia",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.sans));
        beatBox.addSong("Imperial March",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.imperialmarch));
        beatBox.addSong("We Are Number One (Remix)",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.wenumberone));
        beatBox.addSong("Smooth Criminal",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.smoothcrime));
        beatBox.addSong("Space Mountain Theme",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.spacemountain));
        beatBox.addSong("Gas Gas Gas",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.gasgasgas));
        beatBox.addSong("Top 10 Numbers",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.toptennumbers));
        beatBox.addSong("Deja Vu",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.dejavudos));
    }

    @Override
    public void onUpdate() {
        //Updates the toggle state based on the song button's input.
        songToggle.updateToggle(gamepad.getBooleanInput(SONG_BUTTON));

        //If the song toggle is tripped, then toggle the songs.
        if (songToggle.getCurrentState()) {
            //Map the song index to the song name to play.
            String songToPlay = "";
            switch (songNum) {
                case 0:
                    songToPlay = "Spooky Scary Skeletons (Remix)";
                    break;
                case 1:
                    songToPlay = "Dual of Fates";
                    break;
                case 2:
                    songToPlay = "Ra Ra Rasputin";
                    break;
                case 3:
                    songToPlay = "Gaston";
                    break;
                case 4:
                    songToPlay = "Giorno's Theme";
                    break;
                case 5:
                    songToPlay = "Inferno - FIREFORCE OP";
                    break;
                case 6:
                    songToPlay = "Istanbul Not Constantinople";
                    break;
                case 7:
                    songToPlay = "Still Alive";
                    break;
                case 8:
                    songToPlay = "Imperial March";
                    break;
                case 9:
                    songToPlay = "We Are Number One (Remix)";
                    break;
                case 10:
                    songToPlay = "Still Alive (Swing Version)";
                    break;
                case 11:
                    songToPlay = "Space Mountain Theme";
                    break;
                case 12:
                    songToPlay = "Gas Gas Gas";
                    break;
                case 13:
                    songToPlay = "Top 10 Numbers";
                    break;
                case 14:
                    songToPlay = "Deja Vu";
                    break;
            }
            //Stops the currently playing song.
            if(currentSong != null) {
                beatBox.stopSong(currentSong);
            }
            //Plays the chosen song.
            currentSong = songToPlay;
            beatBox.playSong(songToPlay);

            //Increments the song index with a wraparound effect.
            songNum++;
            songNum = songNum % NUMBER_OF_SONGS;

            //Adds the song to the telemetry.
            telemetry.addData("song num", songNum);
            telemetry.update();
        }
    }

    @Override
    public void onStop() {
        //Stops the currently running song.
        beatBox.stopSong(currentSong);
    }
}
