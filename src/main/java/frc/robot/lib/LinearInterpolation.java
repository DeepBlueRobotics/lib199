package frc.robot.lib;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import edu.wpi.first.wpilibj.Filesystem;

public class LinearInterpolation {
    public double[] xs, ys;
    public double[] slopes, intercepts;
    public double minX, maxX;
    public int numPoints = 0;

    // Performs linear interpolation for a strictly monotonically increasing function.
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
                }
            }
            csvParser.close();
            Arrays.sort(xs);
            minX = xs[0];
            maxX = xs[xs.length - 1];
            Arrays.sort(ys);
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

    // Approximates the respective y coordinate for an arbitrary x-coordinate within the data.
    public double calculate(double x) { 
        // Test to see if the data point is within the domain.
        if ((minX <= x) && (x <= maxX)) {
            for (int i = 1; i < xs.length; i++) {
                // Find the closest datapoint and return the y-value using that datapoint's slope and intercept.
                if (xs[i] - x >= 0) { return (slopes[i - 1] * x + intercepts[i - 1]); } 
            }
            // This should never be run, but calculate must return something.
            return 0.0;
        } else {
            System.out.println("Input data is not in the domain.");
            return 0.0;
        }
    }
}