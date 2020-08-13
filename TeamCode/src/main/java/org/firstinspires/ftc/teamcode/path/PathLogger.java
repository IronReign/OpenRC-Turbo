package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.LocationTrack;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class PathLogger {
    private LocationTrack locationTrack;
    private String pathName;
    private PrintWriter out;
    private Scanner in;
    private List<double[]> path;
    private boolean reading;

    public PathLogger(LocationTrack locationTrack) {
        this.locationTrack = locationTrack;
        path = new ArrayList<>();
    }

    public void createPath(String pathName, boolean logCurrentLocation) throws IOException {
        if(pathName.equals(""))
            pathName = String.format("path%d", new File("./paths").listFiles().length);
        this.pathName = pathName;
        out = new PrintWriter(new FileWriter(String.format("./paths/%s.txt", pathName)));
        if(logCurrentLocation)
            addLocation();
        reading = false;
    }

    public boolean addLocation() {
        if(locationTrack.canGetLocation()) {
            path.add(new double[]{locationTrack.getLatitude(), locationTrack.getLongitude()});
            return true;
        }
        return false;
    }

    public void removeLastLocation() {
        if(path.size() > 0)
            path.remove(path.size() - 1);
    }

    public List<double[]> getPath() {
        return path;
    }

    public void savePath() {
        for(double[] waypoint: path)
            out.println(String.format("%f,%f", waypoint[0], waypoint[1]));
    }

    public void loadPath(String pathName) throws IOException {
        in = new Scanner(new BufferedReader(new FileReader(String.format("./paths/%s.txt", pathName))));
        while(in.hasNextLine())
            path.add(new double[] {in.nextDouble(), in.nextDouble()});
        reading = true;
    }

    public List<String> getPaths() {
        List<String> paths = new ArrayList<>();
        for(File file: new File("./paths").listFiles())
            paths.add(file.getName());
        return paths;
    }

    public boolean getReading() {
        return reading;
    }
}
