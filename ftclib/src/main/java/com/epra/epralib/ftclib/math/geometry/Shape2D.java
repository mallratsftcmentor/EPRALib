package com.epra.epralib.ftclib.math.geometry;

/// A interface that represents a shape in 2D space.
public interface Shape2D {

    /// Returns the area of the shape.
    double area();
    /// Returns the perimeter of the shape.
    double perimeter();
    /// Checks if a given point is within the shape.
    boolean checkPoint(Vector point);
}
