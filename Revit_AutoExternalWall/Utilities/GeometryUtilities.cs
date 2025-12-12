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
        /// </summary>
        private static Curve OffsetLine(Line line, double distance, XYZ direction)
        {
            try
            {
                XYZ start = line.GetEndPoint(0);
                XYZ end = line.GetEndPoint(1);

                // Normalize direction to unit vector
                XYZ normalizedDir = direction.Normalize();

                // Calculate offset points
                XYZ offsetStart = start + normalizedDir * distance;
                XYZ offsetEnd = end + normalizedDir * distance;

                // Create offset line
                return Line.CreateBound(offsetStart, offsetEnd);
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Offset an arc segment
        /// </summary>
        private static Curve OffsetArc(Arc arc, double distance, XYZ direction)
        {
            try
            {
                XYZ center = arc.Center;
                double radius = arc.Radius;
                XYZ normal = arc.Normal;

                // Normalize direction
                XYZ normalizedDir = direction.Normalize();

                // Offset the center of the arc
                XYZ offsetCenter = center + normalizedDir * distance;

                // Create new arc with the same radius at the offset position
                XYZ offsetStart = arc.GetEndPoint(0) + normalizedDir * distance;
                XYZ offsetEnd = arc.GetEndPoint(1) + normalizedDir * distance;

                // Create arc through three points
                try
                {
                    Arc offsetArc = Arc.Create(offsetStart, offsetEnd, offsetCenter);
                    return offsetArc;
                }
                catch
                {
                    // Fallback: create arc by center and radius
                    Arc offsetArc = Arc.Create(offsetCenter, radius, arc.XDirection, arc.YDirection, 0, 2 * Math.PI);
                    return offsetArc;
                }
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
