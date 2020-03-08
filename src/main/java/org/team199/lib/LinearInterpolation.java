package org.team199.lib;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Performs linear interpolation. It is assumed that the function has been formatted so that the x value increases from top to bottom.
 */
public class LinearInterpolation {
    private double[] xs;
    private double[] ys;
    private double[] slopes;
    private double[] intercepts;
    private double minX;
    private double maxX;
    private double minY = Double.POSITIVE_INFINITY;
    private double maxY = Double.NEGATIVE_INFINITY;
    private int numPoints = 0;
    private boolean errorShown = false;

    /**
     * Loads linear interpolation data from a csv file. It is assumed that the function has been formatted so that the x value increases from top to bottom.
     * @param filename The name of the file to read from. The name will be assumed to be relative to {@link Filesystem#getDeployDirectory()}
     */
    public LinearInterpolation(String filename) {
        try {      
            CSVParser csvParser = CSVFormat.DEFAULT.parse(new FileReader(Filesystem.getDeployDirectory().toPath().resolve(Paths.get(filename)).toFile()));
            List<CSVRecord> records = csvParser.getRecords();
            numPoints = records.size() - 1;  // Subtract 1 because of the labels.
            // Set the size of the arrays
            xs = new double[numPoints];
            ys = new double[numPoints];
            slopes = new double[numPoints - 1];
            intercepts = new double[numPoints - 1];

            for (int count = 0; count < numPoints + 1; count++) {
                CSVRecord record = records.get(count);
                if (count > 0) {
                    xs[count - 1] = Double.parseDouble(record.get(0));
                    ys[count - 1] = Double.parseDouble(record.get(1));
                    if (ys[count - 1] > maxY) { maxY = ys[count - 1]; }
                    if (ys[count - 1] < minY) { minY = ys[count - 1]; }
                }
            }
            csvParser.close();
            minX = xs[0];
            maxX = xs[xs.length - 1];
            for (int i = 1; i < numPoints; i++) {
                // Linear interpolation (y = mx + b)
                slopes[i - 1] = (ys[i] - ys[i - 1]) / (xs[i] - xs[i - 1]);
                intercepts[i - 1] = ys[i] - slopes[i - 1] * xs[i];
            }
        } catch (FileNotFoundException e) {
            System.out.println("File named " + filename + " not found.");
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Preforms linear interpolation
     * @param x The x value to interpolate
     * @return The interpolated y value
     */
    public double calculate(double x) { 
        // Test to see if the data point is within the domain.
        if ((minX <= x) && (x <= maxX)) {
            for (int i = 1; i < xs.length; i++) {
                // Find the closest datapoint and return the y-value using that datapoint's slope and intercept.
                if (xs[i] - x >= 0) { return (slopes[i - 1] * x + intercepts[i - 1]); } 
            }
        } else if (x > maxX) {
            if (!errorShown) System.out.println("Input data exceeds domain.");
            errorShown = true;
            
            return ys[xs.length - 1];
        } else if (x < minX) {
            if (!errorShown) System.out.println("Input data is less than domain.");
            errorShown = true;
            return ys[0];
        }
        // This should run only in case neither the calculate or outside domain returns run.
        if (!errorShown) System.out.println("There was an unknown issue.");
        errorShown = true;
        return minY;
    }
}