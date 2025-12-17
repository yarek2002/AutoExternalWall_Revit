using Autodesk.Revit.DB;
using Autodesk.Revit.DB.Architecture;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Revit_AutoExternalWall.Utilities
{
    /// <summary>
    /// Data class to hold curve segment information with associated room
    /// </summary>
    public class CurveSegmentData
    {
        public Curve Curve { get; set; }
        public ElementId RoomId { get; set; }
    }

    /// <summary>
    /// Utility class for wall-related operations
    /// </summary>
    public static class WallUtilities
    {
                      /// <summary>
        /// Get a suitable external wall type from the document (Basic Wall only, not Curtain Wall or Stacked Wall)
        /// </summary>
        public static WallType GetExternalWallType(Document doc)
        {
            try
            {
                // Try to find a wall type with "exterior" or "external" in the name, excluding Curtain Walls and Stacked Walls
                FilteredElementCollector collector = new FilteredElementCollector(doc)
                    .OfClass(typeof(WallType));

                foreach (WallType wallType in collector.Cast<WallType>())
                {
                    // Skip Curtain Walls and Stacked Walls - only allow Basic Walls
                    if (wallType.Kind != WallKind.Basic)
                        continue;

                    string name = wallType.Name.ToLower();
                    if (name.Contains("exterior") || name.Contains("external"))
                    {
                        return wallType;
                    }
                }

                // If not found, get the first available Basic Wall type
                foreach (WallType wallType in collector.Cast<WallType>())
                {
                    if (wallType.Kind == WallKind.Basic)
                    {
                        return wallType;
                    }
                }

                return null;
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

                // Compute total offset distance (center-to-center) using shared helper
                double gapDistance = 0.0; // gap in feet (0 = flush)
                double totalOffsetDistance = ComputeCenterOffset(innerWall, wallType, gapDistance);

                // Get wall face orientation to determine offset direction
                XYZ wallFaceNormal = GetWallFaceNormal(innerWall);

                // Create offset curve for external wall
                List<Curve> offsetCurves = GeometryUtilities.OffsetCurve(curve, totalOffsetDistance, wallFaceNormal);

                foreach (Curve offsetCurve in offsetCurves)
                {
                    if (offsetCurve == null || offsetCurve.Length < 0.01)
                        continue;

                    // Invert curve direction so inner face is on the side toward original wall
                    Curve reversedCurve = offsetCurve.CreateReversed();

                    // Создаём внешнюю стену по всей длине исходной стены без дополнительной подрезки
                    Wall externalWall = Wall.Create(doc, reversedCurve, wallType.Id, level.Id, height, 0.0, false, false);

                    if (externalWall != null)
                    {
                        // Do not change the wall "Location Line" parameter here —
                        // changing it can re-interpret the creation curve and shift the wall.
                        // Disable wall joins by setting the "Allow Join" parameter
                        DisableWallJoins(externalWall);
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
        /// Get wall type thickness (from WallType)
        /// </summary>
        private static double GetWallTypeThickness(WallType wallType)
        {
            try
            {
                if (wallType == null)
                    return ConvertMMToFeet(250);

                Parameter thicknessParam = wallType.get_Parameter(BuiltInParameter.WALL_ATTR_WIDTH_PARAM);
                if (thicknessParam != null && thicknessParam.HasValue)
                    return thicknessParam.AsDouble();

                // Try compound structure via reflection (similar to GetWallThickness)
                try
                {
                    object compStructure = wallType?.GetCompoundStructure();
                    if (compStructure != null)
                    {
                        var csType = compStructure.GetType();
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

                return ConvertMMToFeet(250);
            }
            catch
            {
                return ConvertMMToFeet(250);
            }
        }

        /// <summary>
        /// Compute center-line offset (feet) between existing wall and new wall.
        /// Formula: offset = existingThickness/2 + gap + newThickness/2
        /// </summary>
        private static double ComputeCenterOffset(Wall existingWall, WallType newWallType, double gapFeet)
        {
            try
            {
                double existingThickness = GetWallThickness(existingWall);
                double newThickness = GetWallTypeThickness(newWallType);
                return (existingThickness / 2.0) + gapFeet + (newThickness / 2.0);
            }
            catch
            {
                return 0.0;
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

        /// <summary>
        /// Disable wall joins using the Revit API (at both ends of the wall).
        /// This is more reliable than trying to tweak parameters like "Allow Join".
        /// </summary>
        private static void DisableWallJoins(Wall wall)
        {
            if (wall == null)
                return;

            try
            {
                // Revit API call that explicitly disables joins at each end of the wall.
                // 0 = start, 1 = end.
                WallUtils.DisallowWallJoinAtEnd(wall, 0);
                WallUtils.DisallowWallJoinAtEnd(wall, 1);
            }
            catch
            {
                // Ignore – if API is not available in a specific Revit version,
                // wall will behave with default join settings.
            }
        }

        /// <summary>
        /// Trim a candidate wall curve so it does not intersect provided curves
        /// (intended for existing walls).
        /// Only supports straight line curves. Returns the trimmed curve, or null if it becomes too short.
        /// </summary>
        private static Curve TrimCurveAgainstExisting(Curve candidate, IEnumerable<Curve> existingCurves, double trimOffsetFeet = 0.0)
        {
            if (candidate == null || !(candidate is Line candLine))
                return candidate;

            if (existingCurves == null)
                return candidate;

            // Minimum usable length after trimming (in feet)
            const double minLength = 0.5; // ~150 mm

            XYZ start = candLine.GetEndPoint(0);
            XYZ end = candLine.GetEndPoint(1);
            XYZ dir = (end - start).Normalize();

            foreach (Curve existing in existingCurves)
            {
                if (existing == null) continue;

                try
                {
                    SetComparisonResult res = candLine.Intersect(existing, out IntersectionResultArray arr);

                    // Skip if clearly disjoint
                    if (res == SetComparisonResult.Disjoint)
                        continue;

                    // Collect all intersection points we can find
                    List<XYZ> hits = new List<XYZ>();
                    if (arr != null && !arr.IsEmpty)
                    {
                        for (int i = 0; i < arr.Size; i++)
                        {
                            XYZ p = arr.get_Item(i)?.XYZPoint;
                            if (p != null)
                                hits.Add(p);
                        }
                    }

                    // For colinear overlaps Revit may return empty array; fall back to endpoints of the existing curve
                    if (hits.Count == 0 && (res == SetComparisonResult.Overlap || res == SetComparisonResult.Subset || res == SetComparisonResult.Superset))
                    {
                        for (int i = 0; i < 2; i++)
                        {
                            XYZ ep = existing.GetEndPoint(i);
                            if (ep != null)
                                hits.Add(ep);
                        }
                }

                    // Precompute data for existing line in XY
                    if (!(existing is Line exLine))
                        continue;

                    XYZ exA = exLine.GetEndPoint(0);
                    XYZ exB = exLine.GetEndPoint(1);
                    XYZ exDir = (exB - exA).Normalize();
                    // Normal in XY plane (perpendicular to existing wall axis)
                    XYZ exNormal = new XYZ(-exDir.Y, exDir.X, 0.0);

                    foreach (var p in hits)
                    {
                        if (p == null)
                            continue;

                        // Signed distance from intersection to existing wall center line along its normal
                        double dHit = (p - exA).DotProduct(exNormal);

                        // If no trim offset (half-thickness) is provided, just cut at intersection
                        if (Math.Abs(trimOffsetFeet) < 1e-6)
                        {
                            double distStart0 = p.DistanceTo(start);
                            double distEnd0 = p.DistanceTo(end);
                            if (distStart0 <= distEnd0)
                                start = p;
                            else
                                end = p;

                            if (start.DistanceTo(end) < minLength)
                                return null;

                            candLine = Line.CreateBound(start, end);
                            dir = (end - start).Normalize();
                            continue;
                        }

                        // Если пересечение попадает в торец существующей стены – это узел угла,
                        // в этом случае режем ровно по точке пересечения, без смещения на halfThickness.
                        double distToExEnds = Math.Min(p.DistanceTo(exA), p.DistanceTo(exB));
                        const double cornerEndTol = 1e-4; // ~0.03 мм

                        if (distToExEnds < cornerEndTol || Math.Abs(trimOffsetFeet) < 1e-6)
                        {
                            double distStart0 = p.DistanceTo(start);
                            double distEnd0 = p.DistanceTo(end);
                            if (distStart0 <= distEnd0)
                                start = p;
                            else
                                end = p;

                            if (start.DistanceTo(end) < minLength)
                                return null;

                            candLine = Line.CreateBound(start, end);
                            dir = (end - start).Normalize();
                            continue;
                        }

                        // Во всех остальных случаях – сдвиг до внешней грани стены (center +/- halfThickness)
                        double target = (dHit >= 0 ? 1.0 : -1.0) * trimOffsetFeet;

                        // Decide which end to move (closer to intersection along the candidate)
                        double distStart = p.DistanceTo(start);
                        double distEnd = p.DistanceTo(end);
                        bool moveStart = distStart <= distEnd;

                        XYZ basePt = moveStart ? start : end;

                        // Signed distance of base point to existing wall center line
                        double a0 = (basePt - exA).DotProduct(exNormal);
                        double denom = dir.DotProduct(exNormal);

                        if (Math.Abs(denom) < 1e-9)
                        {
                            // Candidate is nearly parallel to existing wall; fall back to cutting at intersection
                            if (moveStart)
                                start = p;
                            else
                                end = p;
                        }
                        else
                        {
                            // Solve for t along candidate direction so that new point has distance "target" to existing wall center line
                            double t = (target - a0) / denom;
                            XYZ newPt = basePt + dir * t;

                            if (moveStart)
                                start = newPt;
                            else
                                end = newPt;
                        }

                        if (start.DistanceTo(end) < minLength)
                            return null;

                        candLine = Line.CreateBound(start, end);
                        dir = (end - start).Normalize();
                    }
                }
                catch
                {
                    // ignore and continue
                }
            }

            return candLine;
        }

        /// <summary>
        /// Trim a candidate wall curve against already created external walls.
        /// То же, что TrimCurveAgainstExisting, но:
        /// - если пересечение попадает прямо в торец новой или существующей внешней стены (выпуклый угол),
        ///   не обрезаем, чтобы стены сходились в вершине;
        /// - иначе обрезаем по внешней грани (на offset половины толщины).
        /// </summary>
        private static Curve TrimCurveAgainstExternalCurves(Curve candidate, IEnumerable<Curve> externalCurves, double trimOffsetFeet = 0.0)
        {
            if (candidate == null || !(candidate is Line candLine))
                return candidate;

            if (externalCurves == null)
                return candidate;

            const double minLength = 0.5;     // ~150 mm
            const double endTol = 1e-4;       // ~0.03 mm, считать точку торцем

            XYZ start = candLine.GetEndPoint(0);
            XYZ end = candLine.GetEndPoint(1);
            XYZ dir = (end - start).Normalize();

            foreach (Curve existing in externalCurves)
            {
                if (existing == null) continue;

                try
                {
                    SetComparisonResult res = candLine.Intersect(existing, out IntersectionResultArray arr);
                    if (res == SetComparisonResult.Disjoint)
                        continue;

                    List<XYZ> hits = new List<XYZ>();
                    if (arr != null && !arr.IsEmpty)
                    {
                        for (int i = 0; i < arr.Size; i++)
                        {
                            XYZ p = arr.get_Item(i)?.XYZPoint;
                            if (p != null)
                                hits.Add(p);
                        }
                    }

                    if (hits.Count == 0 && (res == SetComparisonResult.Overlap || res == SetComparisonResult.Subset || res == SetComparisonResult.Superset))
                    {
                        for (int i = 0; i < 2; i++)
                        {
                            XYZ ep = existing.GetEndPoint(i);
                            if (ep != null)
                                hits.Add(ep);
                        }
                    }

                    if (!(existing is Line exLine))
                        continue;

                    XYZ exA = exLine.GetEndPoint(0);
                    XYZ exB = exLine.GetEndPoint(1);

                    foreach (var p in hits)
                    {
                        if (p == null)
                            continue;

                        // Если пересечение совпадает с торцом новой или существующей внешней стены — выпуклый угол, не режем
                        double hitToExistingEnd = Math.Min(p.DistanceTo(exA), p.DistanceTo(exB));
                        double hitToCandidateEnd = Math.Min(p.DistanceTo(start), p.DistanceTo(end));
                        if (hitToExistingEnd < endTol || hitToCandidateEnd < endTol)
                        {
                            continue;
                        }

                        // Дальше — та же логика, что в TrimCurveAgainstExisting,
                        // только без повторного вычисления нормали и толщины
                        XYZ exDir = (exB - exA).Normalize();
                        XYZ exNormal = new XYZ(-exDir.Y, exDir.X, 0.0);

                        double dHit = (p - exA).DotProduct(exNormal);

                        if (Math.Abs(trimOffsetFeet) < 1e-6)
                        {
                            double distStart0 = p.DistanceTo(start);
                            double distEnd0 = p.DistanceTo(end);
                            if (distStart0 <= distEnd0)
                                start = p;
                            else
                                end = p;
                        }
                        else
                        {
                            double target = (dHit >= 0 ? 1.0 : -1.0) * trimOffsetFeet;

                            double distStart = p.DistanceTo(start);
                            double distEnd = p.DistanceTo(end);
                            bool moveStart = distStart <= distEnd;

                            XYZ basePt = moveStart ? start : end;
                            double a0 = (basePt - exA).DotProduct(exNormal);
                            double denom = dir.DotProduct(exNormal);

                            if (Math.Abs(denom) < 1e-9)
                            {
                                if (moveStart)
                                    start = p;
                                else
                                    end = p;
                            }
                            else
                            {
                                double t = (target - a0) / denom;
                                XYZ newPt = basePt + dir * t;
                                if (moveStart)
                                    start = newPt;
                                else
                                    end = newPt;
                            }
                        }

                        if (start.DistanceTo(end) < minLength)
                            return null;

                        candLine = Line.CreateBound(start, end);
                        dir = (end - start).Normalize();
                    }
                }
                catch
                {
                    // ignore and continue
                }
            }

            return candLine;
        }

        /// <summary>
        /// Collect straight location curves from existing walls in the document,
        /// optionally excluding a specific wall (e.g., the source wall).
        /// </summary>
        private static List<Curve> GetExistingWallCurves(Document doc, Wall excludeWall = null)
        {
            var curves = new List<Curve>();
            try
            {
                var walls = new FilteredElementCollector(doc)
                    .OfClass(typeof(Wall))
                    .Cast<Wall>();

                foreach (var w in walls)
                {
                    if (excludeWall != null && w.Id == excludeWall.Id)
                        continue;

                    if (w.Location is LocationCurve lc && lc.Curve is Line line)
                    {
                        curves.Add(line);
                    }
                }
            }
            catch { }

            return curves;
        }

        /// <summary>
        /// Create external walls for boundary segments of selected rooms that are adjacent to selected walls.
        /// Creates complete walls covering all room boundaries, then splits them at midpoints between rooms.
        /// Returns number of created wall segments after splitting.
        /// </summary>
        public static int CreateExternalWallsFromRooms(Document doc, List<Room> selectedRooms, WallType wallType, List<Wall> selectedWalls = null)
        {
            if (doc == null || selectedRooms == null || selectedRooms.Count == 0 || wallType == null)
                return 0;

            int created = 0;
            List<Curve> createdExternalCurves = new List<Curve>();
            List<Curve> existingWallCurves = GetExistingWallCurves(doc);

            try
            {
                SpatialElementBoundaryOptions opt = new SpatialElementBoundaryOptions();

                // Build set of selected wall IDs for quick lookup
                HashSet<ElementId> selectedWallIds = new HashSet<ElementId>();
                if (selectedWalls != null && selectedWalls.Count > 0)
                {
                    foreach (var wall in selectedWalls)
                    {
                        selectedWallIds.Add(wall.Id);
                    }
                }

                // Group boundary segments by their source wall
                Dictionary<ElementId, List<CurveSegmentData>> segmentsByWall =
                    new Dictionary<ElementId, List<CurveSegmentData>>();

                foreach (Room room in selectedRooms)
                {
                    var loops = room.GetBoundarySegments(opt);
                    if (loops == null)
                        continue;

                    foreach (var loop in loops)
                    {
                        if (loop == null) continue;
                        foreach (var seg in loop)
                        {
                            if (seg == null) continue;

                            ElementId boundaryId = seg.ElementId;

                            // Only process if this segment belongs to a selected wall (or if no walls were selected)
                            if (selectedWalls != null && selectedWalls.Count > 0 && !selectedWallIds.Contains(boundaryId))
                                continue;

                            Element boundaryElem = doc.GetElement(boundaryId);
                            if (boundaryElem is Wall wall)
                            {
                                Curve innerCurve = seg.GetCurve();
                                if (innerCurve == null) continue;

                                CurveSegmentData segmentData = new CurveSegmentData
                                {
                                    Curve = innerCurve,
                                    RoomId = room.Id
                                };

                                if (!segmentsByWall.TryGetValue(boundaryId, out var list))
                                {
                                    list = new List<CurveSegmentData>();
                                    segmentsByWall[boundaryId] = list;
                                }

                                list.Add(segmentData);
                            }
                        }
                    }
                }

                // For each wall, create one external wall covering all boundary curves, then split at room midpoints
                foreach (var kvp in segmentsByWall)
                {
                    ElementId wallId = kvp.Key;
                    Element boundaryElem = doc.GetElement(wallId);
                    if (!(boundaryElem is Wall innerWall))
                        continue;

                    var segmentDatas = kvp.Value;
                    if (segmentDatas.Count == 0)
                        continue;

                    // Find split points: midpoints between rooms
                    List<double> splitParams = CalculateSplitPoints(innerWall, segmentDatas);

                    // Get curve segments based on split points
                    List<Curve> wallSegments = GetWallSegments(innerWall, segmentDatas, splitParams);

                    // Create external walls for each segment
                    foreach (Curve segment in wallSegments)
                    {
                        Wall externalWall = CreateExternalWallAlongCurveSingle(doc, innerWall, segment, wallType, createdExternalCurves, existingWallCurves);
                        if (externalWall != null)
                        {
                            created++;
                        }
                    }
                }
            }
            catch { }

            return created;
        }

        /// <summary>
        /// Create an external wall along a specific curve segment that belongs to an existing wall.
        /// Returns number of created walls (0 or 1 normally).
        /// </summary>
        public static int CreateExternalWallAlongCurve(Document doc, Wall innerWall, Curve innerCurve, WallType wallType)
        {
            if (doc == null || innerWall == null || innerCurve == null || wallType == null)
                return 0;

            try
            {
                Level level = GetWallLevel(innerWall);
                double height = GetWallHeight(innerWall);
                if (level == null) return 0;

                double totalOffsetDistance = ComputeCenterOffset(innerWall, wallType, 0.0);

                XYZ wallFaceNormal = GetWallFaceNormal(innerWall);

                List<Curve> offsetCurves = GeometryUtilities.OffsetCurve(innerCurve, totalOffsetDistance, wallFaceNormal);
                int created = 0;
                List<Curve> createdExternalCurves = new List<Curve>();
                List<Curve> existingWallCurves = GetExistingWallCurves(doc, innerWall);
                double existingHalfThickness = GetWallThickness(innerWall) / 2.0;
                double externalHalfThickness = GetWallTypeThickness(wallType) / 2.0;

                foreach (Curve offsetCurve in offsetCurves)
                {
                    if (offsetCurve == null || offsetCurve.Length < 0.01) continue;

                    Curve reversed = offsetCurve.CreateReversed();
                    // 1) trim against existing walls
                    Curve trimmed = TrimCurveAgainstExisting(reversed, existingWallCurves, existingHalfThickness);
                    if (trimmed == null)
                        continue;

                    // 2) trim against already created external walls (не режем выпуклые углы)
                    // На внешних углах не укорачиваем по толщине новой стены — тянем до угла
                    trimmed = TrimCurveAgainstExternalCurves(trimmed, createdExternalCurves, 0.0);
                    if (trimmed == null)
                        continue;

                    Wall externalWall = Wall.Create(doc, trimmed, wallType.Id, level.Id, height, 0.0, false, false);
                    if (externalWall != null)
                    {
                        // Keep created wall as-is (curve used during creation should be the
                        // desired location line). Avoid setting "location line" parameter
                        // programmatically which can move the wall unexpectedly.
                        DisableWallJoins(externalWall);
                        CopyWallProperties(innerWall, externalWall);
                        createdExternalCurves.Add(trimmed);
                        created++;
                    }
                }

                return created;
            }
            catch { return 0; }
        }

        /// <summary>
        /// Merge a set of curves that lie along a straight wall into longer continuous
        /// line segments. Returns the merged curves; if the wall is not straight or
        /// merging fails, returns the original curves.
        /// </summary>
        private static List<Curve> MergeCurvesAlongWall(Wall wall, List<Curve> curves)
        {
            var result = new List<Curve>();
            if (wall == null || curves == null || curves.Count == 0)
                return result;

            try
            {
                LocationCurve loc = wall.Location as LocationCurve;
                if (loc == null || loc.Curve == null)
                    return new List<Curve>(curves);

                // Only attempt merging for straight walls (Line location)
                if (!(loc.Curve is Line wallLine))
                    return new List<Curve>(curves);

                XYZ wallStart = wallLine.GetEndPoint(0);
                XYZ wallEnd = wallLine.GetEndPoint(1);
                XYZ dir = (wallEnd - wallStart).Normalize();

                // Convert each curve to an interval along the wall direction
                var intervals = new List<Tuple<double, double>>();
                foreach (var c in curves)
                {
                    if (c == null) continue;
                    XYZ p0 = c.GetEndPoint(0);
                    XYZ p1 = c.GetEndPoint(1);
                    double t0 = (p0 - wallStart).DotProduct(dir);
                    double t1 = (p1 - wallStart).DotProduct(dir);
                    double a = Math.Min(t0, t1);
                    double b = Math.Max(t0, t1);
                    intervals.Add(Tuple.Create(a, b));
                }

                if (intervals.Count == 0)
                    return result;

                intervals.Sort((x, y) => x.Item1.CompareTo(y.Item1));
                // Allow merging across small gaps (partitions) so external walls
                // created for the same source wall become continuous.
                // Tolerance in feet; adjust if necessary (0.5 ft ~= 150 mm).
                double tol = 0.5;

                double curA = intervals[0].Item1;
                double curB = intervals[0].Item2;

                for (int i = 1; i < intervals.Count; ++i)
                {
                    var it = intervals[i];
                    if (it.Item1 <= curB + tol)
                    {
                        // overlapping or adjacent - extend
                        curB = Math.Max(curB, it.Item2);
                    }
                    else
                    {
                        XYZ aPt = wallStart + (dir * curA);
                        XYZ bPt = wallStart + (dir * curB);
                        result.Add(Line.CreateBound(aPt, bPt));
                        curA = it.Item1;
                        curB = it.Item2;
                    }
                }

                // add final interval
                XYZ lastA = wallStart + (dir * curA);
                XYZ lastB = wallStart + (dir * curB);
                result.Add(Line.CreateBound(lastA, lastB));

                return result;
            }
            catch
            {
                return new List<Curve>(curves);
            }
        }

        /// <summary>
        /// Get a curve that encompasses all given curves along a wall, creating a continuous curve covering the full span.
        /// </summary>
        private static Curve GetEncompassingCurve(Wall wall, List<Curve> curves)
        {
            if (wall == null || curves == null || curves.Count == 0)
                return null;

            try
            {
                LocationCurve loc = wall.Location as LocationCurve;
                if (loc == null || loc.Curve == null || !(loc.Curve is Line wallLine))
                    return null; // Only for straight walls

                XYZ wallStart = wallLine.GetEndPoint(0);
                XYZ wallEnd = wallLine.GetEndPoint(1);
                XYZ dir = (wallEnd - wallStart).Normalize();

                // Find min and max along the wall direction
                double minT = double.MaxValue;
                double maxT = double.MinValue;

                foreach (var c in curves)
                {
                    if (c == null) continue;
                    XYZ p0 = c.GetEndPoint(0);
                    XYZ p1 = c.GetEndPoint(1);
                    double t0 = (p0 - wallStart).DotProduct(dir);
                    double t1 = (p1 - wallStart).DotProduct(dir);
                    minT = Math.Min(minT, Math.Min(t0, t1));
                    maxT = Math.Max(maxT, Math.Max(t0, t1));
                }

                if (minT >= maxT)
                    return null;

                XYZ startPt = wallStart + dir * minT;
                XYZ endPt = wallStart + dir * maxT;

                return Line.CreateBound(startPt, endPt);
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Create a single external wall along the curve.
        /// Returns the created Wall or null if failed.
        /// </summary>
        private static Wall CreateExternalWallAlongCurveSingle(Document doc, Wall innerWall, Curve innerCurve, WallType wallType, List<Curve> existingExternalCurves = null, List<Curve> existingWallCurves = null)
        {
            if (doc == null || innerWall == null || innerCurve == null || wallType == null)
                return null;

            try
            {
                Level level = GetWallLevel(innerWall);
                double height = GetWallHeight(innerWall);
                if (level == null)
                    return null;

                double existingThickness = GetWallThickness(innerWall);
                double newThickness = GetWallTypeThickness(wallType);
                double totalOffsetDistance = (existingThickness/ 2.0) + (newThickness / 2.0);

                XYZ wallFaceNormal = GetWallFaceNormal(innerWall);

                List<Curve> offsetCurves = GeometryUtilities.OffsetCurve(innerCurve, totalOffsetDistance, wallFaceNormal);
                if (offsetCurves.Count == 0 || offsetCurves[0] == null)
                    return null;

                Curve offsetCurve = offsetCurves[0];
                Curve reversed = offsetCurve.CreateReversed();
                // 1) trim against existing walls
                List<Curve> existingWalls = existingWallCurves ?? new List<Curve>();
                Curve trimmed = TrimCurveAgainstExisting(reversed, existingWalls, existingThickness / 2.0);
                if (trimmed == null)
                    return null;

                // 2) trim against already created external walls
                if (existingExternalCurves != null && existingExternalCurves.Count > 0)
                {
                    trimmed = TrimCurveAgainstExternalCurves(trimmed, existingExternalCurves, newThickness / 2.0);
                    if (trimmed == null)
                        return null;
                }
                if (trimmed == null)
                    return null;

                Wall externalWall = Wall.Create(doc, trimmed, wallType.Id, level.Id, height, 0.0, false, false);
                if (externalWall != null)
                {
                    DisableWallJoins(externalWall);
                    CopyWallProperties(innerWall, externalWall);
                    existingExternalCurves?.Add(trimmed);
                }

                return externalWall;
            }
            catch { return null; }
        }

        /// <summary>
        /// Calculate split points (midpoints) between room boundaries along the wall.
        /// </summary>
        private static List<double> CalculateSplitPoints(Wall wall, List<CurveSegmentData> segmentDatas)
        {
            var result = new List<double>();

            if (wall == null || segmentDatas == null || segmentDatas.Count == 0)
                return result;

            try
            {
                LocationCurve loc = wall.Location as LocationCurve;
                if (loc == null || loc.Curve == null || !(loc.Curve is Line wallLine))
                    return result;

                XYZ wallStart = wallLine.GetEndPoint(0);
                XYZ wallEnd = wallLine.GetEndPoint(1);
                XYZ dir = (wallEnd - wallStart).Normalize();

                // Group by room and find intervals
                var roomIntervals = new Dictionary<ElementId, Tuple<double, double>>();

                foreach (var segmentData in segmentDatas)
                {
                    if (segmentData.Curve == null) continue;

                    XYZ p0 = segmentData.Curve.GetEndPoint(0);
                    XYZ p1 = segmentData.Curve.GetEndPoint(1);
                    double t0 = (p0 - wallStart).DotProduct(dir);
                    double t1 = (p1 - wallStart).DotProduct(dir);
                    double minT = Math.Min(t0, t1);
                    double maxT = Math.Max(t0, t1);

                    ElementId roomId = segmentData.RoomId;
                    if (roomIntervals.TryGetValue(roomId, out var existing))
                    {
                        minT = Math.Min(minT, existing.Item1);
                        maxT = Math.Max(maxT, existing.Item2);
                    }
                    roomIntervals[roomId] = Tuple.Create(minT, maxT);
                }

                // Sort intervals by start position
                var sortedIntervals = roomIntervals.Values.OrderBy(t => t.Item1).ToList();

                // Find midpoints between consecutive intervals
                for (int i = 0; i < sortedIntervals.Count - 1; i++)
                {
                    double endOfFirst = sortedIntervals[i].Item2;
                    double startOfNext = sortedIntervals[i + 1].Item1;

                    // Check if there's a gap or overlap; only split if adjacent or close
                    double midpoint = (endOfFirst + startOfNext) / 2.0;
                    result.Add(midpoint);
                }

                return result;
            }
            catch { return result; }
        }



        /// <summary>
        /// Get wall curve segments based on split points along the encompassing curve.
        /// </summary>
        private static List<Curve> GetWallSegments(Wall wall, List<CurveSegmentData> segmentDatas, List<double> splitParams)
        {
            var result = new List<Curve>();

            if (wall == null || segmentDatas == null || segmentDatas.Count == 0)
                return result;

            try
            {
                LocationCurve loc = wall.Location as LocationCurve;
                if (loc == null || loc.Curve == null || !(loc.Curve is Line wallLine))
                    return result;

                XYZ wallStart = wallLine.GetEndPoint(0);
                XYZ wallEnd = wallLine.GetEndPoint(1);
                XYZ dir = (wallEnd - wallStart).Normalize();
                double wallLength = wallStart.DistanceTo(wallEnd);
                // Насколько "вытащить" сегменты к внешним углам: ширина граничной стены
                double extendBy = GetWallThickness(wall);

                // Find the encompassing min and max from segmentDatas
                double minT = double.MaxValue;
                double maxT = double.MinValue;

                foreach (var segmentData in segmentDatas)
                {
                    if (segmentData.Curve == null) continue;

                    XYZ p0 = segmentData.Curve.GetEndPoint(0);
                    XYZ p1 = segmentData.Curve.GetEndPoint(1);
                    double t0 = (p0 - wallStart).DotProduct(dir);
                    double t1 = (p1 - wallStart).DotProduct(dir);
                    minT = Math.Min(minT, Math.Min(t0, t1));
                    maxT = Math.Max(maxT, Math.Max(t0, t1));
                }

                if (minT >= maxT)
                    return result;

                // Build split points: include min, splits, max
                var splitPoints = new List<double> { minT };
                splitPoints.AddRange(splitParams.Where(p => p > minT && p < maxT));
                splitPoints.Add(maxT);
                splitPoints.Sort();

                // Create segments between consecutive split points
                for (int i = 0; i < splitPoints.Count - 1; i++)
                {
                    double startT = splitPoints[i];
                    double endT = splitPoints[i + 1];

                    // Для крайних сегментов вытягиваем их к внешним углам здания,
                    // чтобы внешняя стена доходила до реального конца стены,
                    // а не обрывалась на границе помещения.
                    if (i == 0)
                    {
                        startT = Math.Max(0.0, startT - extendBy);
                    }
                    if (i == splitPoints.Count - 2)
                    {
                        endT = Math.Min(wallLength, endT + extendBy);
                    }

                    if (endT > startT + 0.01) // Ignore very small segments
                    {
                        XYZ startPt = wallStart + dir * startT;
                        XYZ endPt = wallStart + dir * endT;
                        Curve segment = Line.CreateBound(startPt, endPt);
                        result.Add(segment);
                    }
                }

                return result;
            }
            catch { return result; }
        }
    }
}
