using Autodesk.Revit.DB;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Revit_AutoExternalWall.Utilities
{
    /// <summary>
    /// Utility class for wall-related operations
    /// </summary>
    public static class WallUtilities
    {
        /// <summary>
        /// Get a suitable external wall type from the document
        /// </summary>
        public static WallType GetExternalWallType(Document doc)
        {
            try
            {
                // Try to find a wall type with "exterior" or "external" in the name
                FilteredElementCollector collector = new FilteredElementCollector(doc)
                    .OfClass(typeof(WallType));

                foreach (WallType wallType in collector.Cast<WallType>())
                {
                    string name = wallType.Name.ToLower();
                    if (name.Contains("exterior") || name.Contains("external"))
                    {
                        return wallType;
                    }
                }

                // If not found, get the first available wall type
                WallType defaultWallType = collector.FirstOrDefault() as WallType;
                return defaultWallType;
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Create an external wall parallel to the selected wall
        /// The external wall will be placed adjacent to the outer face of the selected wall
        /// </summary>
        public static int CreateExternalWall(Document doc, Wall innerWall, WallType wallType)
        {
            if (innerWall == null || wallType == null)
                return 0;

            int wallsCreated = 0;

            try
            {
                // Get wall location
                LocationCurve locationCurve = innerWall.Location as LocationCurve;
                if (locationCurve == null || locationCurve.Curve == null)
                    return 0;

                Curve curve = locationCurve.Curve;

                // Get wall properties
                double height = GetWallHeight(innerWall);
                Level level = GetWallLevel(innerWall);

                if (level == null)
                    return 0;

                // Get wall thickness and calculate total offset distance
                // Offset = wall thickness + gap (200mm)
                double wallThickness = GetWallThickness(innerWall);
                double gapDistance = ConvertMMToFeet(200); // 200mm gap
                double totalOffsetDistance = wallThickness + gapDistance;

                // Get wall face orientation to determine offset direction
                XYZ wallFaceNormal = GetWallFaceNormal(innerWall);

                // Create offset curve for external wall
                List<Curve> offsetCurves = GeometryUtilities.OffsetCurve(curve, totalOffsetDistance, wallFaceNormal);

                foreach (Curve offsetCurve in offsetCurves)
                {
                    if (offsetCurve == null || offsetCurve.Length < 0.01)
                        continue;

                    // Create wall (use overload with offset, flip and structural parameters)
                    Wall externalWall = Wall.Create(doc, offsetCurve, wallType.Id, level.Id, height, 0.0, false, false);

                    if (externalWall != null)
                    {
                        // Copy properties from inner wall to external wall
                        CopyWallProperties(innerWall, externalWall);
                        wallsCreated++;
                    }
                }
            }
            catch (Exception ex)
            {
                throw new Exception($"Error creating external wall: {ex.Message}");
            }

            return wallsCreated;
        }

        /// <summary>
        /// Get wall height
        /// </summary>
        private static double GetWallHeight(Wall wall)
        {
            try
            {
                Parameter heightParam = wall.get_Parameter(BuiltInParameter.WALL_USER_HEIGHT_PARAM);
                if (heightParam != null && heightParam.HasValue)
                    return heightParam.AsDouble();

                // Alternative: use element range
                BoundingBoxXYZ bbox = wall.get_BoundingBox(null);
                if (bbox != null)
                    return bbox.Max.Z - bbox.Min.Z;

                return 10; // Default height (feet)
            }
            catch
            {
                return 10; // Default height
            }
        }

        /// <summary>
        /// Get wall level
        /// </summary>
        private static Level GetWallLevel(Wall wall)
        {
            try
            {
                ElementId levelId = wall.LevelId;
                if (levelId != null && levelId != ElementId.InvalidElementId)
                {
                    return wall.Document.GetElement(levelId) as Level;
                }
            }
            catch { }

            // Fallback: get first level
            FilteredElementCollector collector = new FilteredElementCollector(wall.Document)
                .OfClass(typeof(Level));

            return collector.FirstOrDefault() as Level;
        }

        /// <summary>
        /// Convert millimeters to feet
        /// 1 foot = 304.8 mm
        /// </summary>
        private static double ConvertMMToFeet(double millimeters)
        {
            return millimeters / 304.8;
        }

        /// <summary>
        /// Get wall thickness
        /// </summary>
        private static double GetWallThickness(Wall wall)
        {
            try
            {
                // Try to get thickness from wall type
                WallType wallType = wall.WallType;
                if (wallType != null)
                {
                    Parameter thicknessParam = wallType.get_Parameter(BuiltInParameter.WALL_ATTR_WIDTH_PARAM);
                    if (thicknessParam != null && thicknessParam.HasValue)
                    {
                        return thicknessParam.AsDouble();
                    }
                }

                // Alternative: try to get from compound structure via reflection (works across API variations)
                try
                {
                    object compStructure = wallType?.GetCompoundStructure();
                    if (compStructure != null)
                    {
                        var csType = compStructure.GetType();

                        // Try to get a collection of layers from property 'Layers' or method 'GetLayers'
                        object layersObj = null;
                        var prop = csType.GetProperty("Layers");
                        if (prop != null)
                            layersObj = prop.GetValue(compStructure);
                        else
                        {
                            var m = csType.GetMethod("GetLayers");
                            if (m != null)
                                layersObj = m.Invoke(compStructure, null);
                        }

                        if (layersObj is System.Collections.IEnumerable layersEnum)
                        {
                            double totalThickness = 0;
                            foreach (var layerObj in layersEnum)
                            {
                                if (layerObj == null) continue;
                                var layerType = layerObj.GetType();

                                // Try common thickness/width property names
                                object thicknessVal = layerType.GetProperty("Thickness")?.GetValue(layerObj)
                                    ?? layerType.GetProperty("Width")?.GetValue(layerObj);

                                if (thicknessVal is double td)
                                    totalThickness += td;
                                else if (thicknessVal is float tf)
                                    totalThickness += tf;
                                else if (thicknessVal is int ti)
                                    totalThickness += ti;
                            }

                            if (totalThickness > 0)
                                return totalThickness;
                        }
                    }
                }
                catch { }

                // Fallback: estimate from bounding box
                BoundingBoxXYZ bbox = wall.get_BoundingBox(null);
                if (bbox != null)
                {
                    // Approximate thickness as smallest horizontal dimension
                    double xSize = bbox.Max.X - bbox.Min.X;
                    double ySize = bbox.Max.Y - bbox.Min.Y;
                    return Math.Min(xSize, ySize);
                }

                return ConvertMMToFeet(250); // Default 250mm thickness
            }
            catch
            {
                return ConvertMMToFeet(250); // Default 250mm thickness
            }
        }

        /// <summary>
        /// Get the normal vector of the outer wall face
        /// </summary>
        private static XYZ GetWallFaceNormal(Wall wall)
        {
            try
            {
                LocationCurve locCurve = wall.Location as LocationCurve;
                if (locCurve == null)
                    return XYZ.BasisY;

                Curve curve = locCurve.Curve;
                if (curve == null)
                    return XYZ.BasisY;

                // Get tangent vector along the wall
                XYZ startPoint = curve.GetEndPoint(0);
                XYZ tangent = (curve.GetEndPoint(1) - startPoint).Normalize();

                // Calculate perpendicular vector in XY plane (rotate 90 degrees counterclockwise)
                // This points outward from the wall
                XYZ normal = new XYZ(-tangent.Y, tangent.X, 0);

                // Normalize and return
                if (normal.GetLength() > 0)
                    return normal.Normalize();

                return XYZ.BasisY;
            }
            catch
            {
                return XYZ.BasisY;
            }
        }

        /// <summary>
        /// Copy relevant properties from one wall to another
        /// </summary>
        private static void CopyWallProperties(Wall source, Wall target)
        {
            try
            {
                // Try to copy a parameter related to "construction" by name (some Revit versions lack a BuiltInParameter)
                Parameter constructTypeSource = FindParameterByNameContains(source, "construction");
                Parameter constructTypeTarget = FindParameterByNameContains(target, "construction");

                if (constructTypeSource != null && constructTypeTarget != null && constructTypeSource.HasValue)
                {
                    switch (constructTypeSource.StorageType)
                    {
                        case StorageType.Integer:
                            constructTypeTarget.Set(constructTypeSource.AsInteger());
                            break;
                        case StorageType.Double:
                            constructTypeTarget.Set(constructTypeSource.AsDouble());
                            break;
                        case StorageType.String:
                            constructTypeTarget.Set(constructTypeSource.AsString());
                            break;
                        case StorageType.ElementId:
                            constructTypeTarget.Set(constructTypeSource.AsElementId());
                            break;
                    }
                }
            }
            catch { }
        }

        /// <summary>
        /// Find a parameter on an element whose name contains the provided substring (case-insensitive)
        /// </summary>
        private static Parameter FindParameterByNameContains(Element el, string namePart)
        {
            if (el == null || string.IsNullOrEmpty(namePart))
                return null;

            string lower = namePart.ToLowerInvariant();
            foreach (Parameter p in el.Parameters)
            {
                if (p.Definition != null && p.Definition.Name != null && p.Definition.Name.ToLowerInvariant().Contains(lower))
                    return p;
            }

            return null;
        }
    }
}
