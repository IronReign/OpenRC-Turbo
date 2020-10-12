package org.firstinspires.ftc.teamcode.path;

import android.location.Location;

/* this is an endless arc path on the front lawn of 9512 folkstone 75220
* the robot ping-pongs between the endpoints*/

public class pingpongpath extends NavPath {

    private int index = -1;
    private boolean forward = true;
    double[] points = {-96.84058094227957,32.86939561197352,-96.84055382713342,32.86937532871967,-96.84053815116386,32.86935077528167,-96.84053137233843,32.86932551015268,-96.84053391430362,32.86929526315991,-96.84054492973115,32.86927106555252,-96.84056568962939,32.8692486471622,-96.84058899158062,32.86923441323794,-96.84062161433488,32.86922551699113};
    @Override
    public int getSegment() {
        return 0;
    }

    @Override
    public void setSegment(int segment) {

    }

    @Override
    public int getindex() {
        return index;
    }

    public Location getNext(){
        if ((index+1) * 2 == points.length) forward = false;
        if (forward) {
            index++;
            loc.setLongitude(points[index*2]);
            loc.setLatitude(points[index*2+1]);
        }
        else { //doubling back to start (index 0)
            index--;
            loc.setLongitude(points[index*2]);
            loc.setLatitude(points[index*2+1]);
            if (index == 0) {
                index = -1;
                segment++;
                forward = true;
                return loc;
            }
        }
        return loc;

}

public boolean isDone() {
    return false;
} //this path is endless

    @Override
    public String getPathName() {
        return "PingPongU";
    }

}
