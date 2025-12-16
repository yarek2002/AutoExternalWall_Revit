using Autodesk.Revit.DB;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Revit_AutoExternalWall.Utilities
{
    /// <summary>
    /// Utility class for geometry operations
    /// </summary>
    public static class GeometryUtilities
    {
        /// <summary>
        /// Offset a curve by a specified distance in the direction of the normal
        /// </summary>
        public static List<Curve> OffsetCurve(Curve curve, double distance, XYZ direction)
        {
            List<Curve> offsetCurves = new List<Curve>();

            try
            {
                if (curve is Line line)
                {
                    Curve offsetCurve = OffsetLine(line, distance, direction);
                    if (offsetCurve != null)
                        offsetCurves.Add(offsetCurve);
                }
                else if (curve is Arc arc)
                {
                    Curve offsetCurve = OffsetArc(arc, distance, direction);
                    if (offsetCurve != null)
                        offsetCurves.Add(offsetCurve);
                }
                else
                {
                    // For other curve types, try generic offset
                    Curve offsetCurve = OffsetGenericCurve(curve, distance, direction);
                    if (offsetCurve != null)
                        offsetCurves.Add(offsetCurve);
                }
            }
            catch (Exception ex)
            {
                throw new Exception($"Error offsetting curve: {ex.Message}");
            }

            return offsetCurves;
        }

        /// <summary>
        /// Offset a line segment in the direction of the given normal
        /// Extends the line by 1000 feet in both directions before offsetting to allow walls to meet at external corners
        /// </summary>
        private static Curve OffsetLine(Line line, double distance, XYZ direction)
        {
            try
            {
                XYZ start = line.GetEndPoint(0);
                XYZ end = line.GetEndPoint(1);

                // Get line direction
                XYZ dir = (end - start).Normalize();

                // Extend the line by 1000 feet in both directions to ensure it reaches external corners
                double extension = 1000.0; // feet - sufficient for building scales
                XYZ extendedStart = start - dir * extension;
                XYZ extendedEnd = end + dir * extension;

                // Normalize offset direction
                XYZ normalizedDir = direction.Normalize();

                // Calculate offset points on the extended line
                XYZ offsetStart = extendedStart + normalizedDir * distance;
                XYZ offsetEnd = extendedEnd + normalizedDir * distance;

                // Create offset line
                return Line.CreateBound(offsetStart, offsetEnd);
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Offset an arc segment by sampling points and creating an arc through offset points
        /// </summary>
        private static Curve OffsetArc(Arc arc, double distance, XYZ direction)
        {
            try
            {
                XYZ normalizedDir = direction.Normalize();

                // Sample three points on the arc: start, mid, end
                XYZ start = arc.GetEndPoint(0);
                XYZ end = arc.GetEndPoint(1);
                double midParam = 0.5;
                XYZ mid = arc.Evaluate(midParam, false);

                // Offset sampled points by the given direction
                XYZ offsetStart = start + normalizedDir * distance;
                XYZ offsetMid = mid + normalizedDir * distance;
                XYZ offsetEnd = end + normalizedDir * distance;

                // Create an arc through the three offset points
                Arc offsetArc = null;
                try
                {
                    offsetArc = Arc.Create(offsetStart, offsetEnd, offsetMid);
                }
                catch
                {
                    // If creation fails, fallback to a simple offset line between start and end
                    return Line.CreateBound(offsetStart, offsetEnd);
                }

                return offsetArc;
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Offset a generic curve
        /// </summary>
        private static Curve OffsetGenericCurve(Curve curve, double distance, XYZ direction)
        {
            try
            {
                XYZ normalizedDir = direction.Normalize();

                // Get curve points and offset them
                double curveLength = curve.Length;
                int numberOfPoints = Math.Max(10, (int)(curveLength / 1.0));

                List<XYZ> offsetPoints = new List<XYZ>();

                for (int i = 0; i <= numberOfPoints; i++)
                {
                    double parameter = i / (double)numberOfPoints;
                    XYZ curvePoint = curve.Evaluate(parameter, false);

                    // Apply offset in the given direction
                    XYZ offsetPoint = curvePoint + normalizedDir * distance;
                    offsetPoints.Add(offsetPoint);
                }

                // Create line from first to last offset point
                if (offsetPoints.Count >= 2)
                {
                    return Line.CreateBound(offsetPoints[0], offsetPoints[offsetPoints.Count - 1]);
                }

                return null;
            }
            catch
            {
                return null;
            }
        }
    }
}
