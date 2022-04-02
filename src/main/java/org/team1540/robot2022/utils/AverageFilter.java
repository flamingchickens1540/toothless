package org.team1540.robot2022.utils;

import java.util.LinkedList;

/**
 * Class for filtering number data as a rolling or fixed average.
 */
public class AverageFilter {
    private final LinkedList<Double> buffer;
    int size;

    public AverageFilter(int size) {
        this.buffer = new LinkedList<>();
        this.size = size;
    }

    public void add(double item) {
        buffer.add(item);
        if (buffer.size() > size) {
            buffer.remove(0);
        }
    }

    public double getAverage() {
        double avg = 0;
        for (double num : buffer) {
            avg += num;
        }
        return avg / buffer.size();
    }

    public void clear() {
        buffer.clear();
    }
}
