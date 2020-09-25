package org.firstinspires.ftc.teamcode.path;

import android.location.Location;

/* this is an endless path at the current location of the robot
* it needs to be initialized */

public class staypath extends NavPath {
 //initial home position is 9512 folkstone middle of front walkway
    double lon = -96.84057999999999;
    double lat = 32.86929613306522;

    public void setLocation (double lat, double lon){
        this.lat = lat;
        this.lon = lon;
    }

    @Override
    public int getSegment() {
        return 0;
    }

    @Override
    public void setSegment(int segment) {

    }

    @Override
    public int getindex() {
        return 0;
    }

    public Location getNext(){

        Location loc = new Location("");
        loc.setLongitude(lon);
        loc.setLatitude(lat);
        return loc;
    }

    public boolean isDone() {
        return false;
    } //this path is endless

    @Override
    public String getPathName() {
        return "staying";
    }

}