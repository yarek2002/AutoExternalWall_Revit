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
        /// Helper to store wall with its straight location curve.
        /// Используется для корректной подрезки в углах с учётом толщины обеих стен.
        /// </summary>
        private class WallCurveInfo
        {
            public Wall Wall { get; set; }
            public Curve Curve { get; set; }
        }

        /// <summary>
        /// Кандидат внешней стены, который мы сначала считаем геометрически,
        /// а затем создаём уже готовые стены одной пачкой.
        /// </summary>
        private class ExternalWallCandidate
        {
            public Wall InnerWall { get; set; }
            public Curve Curve { get; set; }
        }
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
        /// Create external walls for a set of existing walls, taking into account
        /// their mutual intersections so that новые внешние стены сходятся в общей
        /// точке, но угол остаётся «открытым» (join'ы отключены).
        /// </summary>
        public static int CreateExternalWallsFromExistingWalls(Document doc, List<Wall> walls, WallType wallType)
        {
            if (doc == null || walls == null || walls.Count == 0 || wallType == null)
                return 0;

            var candidates = new List<ExternalWallCandidate>();
            int created = 0;

            try
            {
                foreach (var innerWall in walls)
                {
                    if (innerWall == null)
                        continue;

                    if (!(innerWall.Location is LocationCurve lc) || lc.Curve == null)
                        continue;

                    Curve baseCurve = lc.Curve;
                    if (!(baseCurve is Line) || baseCurve.Length < 0.01)
                        continue; // для простоты работаем только с прямыми стенами

                    double existingThickness = GetWallThickness(innerWall);
                    double newThickness = GetWallTypeThickness(wallType);
                    double totalOffsetDistance = (existingThickness / 2.0) + (newThickness / 2.0);

                    XYZ wallFaceNormal = GetWallFaceNormal(innerWall);
                    var offsetCurves = GeometryUtilities.OffsetCurve(baseCurve, totalOffsetDistance, wallFaceNormal);
                    if (offsetCurves == null || offsetCurves.Count == 0)
                        continue;

                    Curve offset = offsetCurves[0];
                    if (offset == null || offset.Length < 0.01)
                        continue;

                    Curve reversed = offset.CreateReversed();

                    candidates.Add(new ExternalWallCandidate
                    {
                        InnerWall = innerWall,
                        Curve = reversed
                    });
                }

                // Стягиваем все кандидаты к точкам пересечения их осей,
                // учитывая половину толщины создаваемой внешней стены.
                double externalHalfThickness = GetWallTypeThickness(wallType) / 2.0;
                AdjustExternalCandidatesAtIntersections(candidates, externalHalfThickness);

                // Создаём реальные стены
                foreach (var cand in candidates)
                {
                    if (cand?.Curve == null || cand.Curve.Length < 0.01 || cand.InnerWall == null)
                        continue;

                    Level level = GetWallLevel(cand.InnerWall);
                    double height = GetWallHeight(cand.InnerWall);
                    if (level == null)
                        continue;

                    Wall externalWall = Wall.Create(doc, cand.Curve, wallType.Id, level.Id, height, 0.0, false, false);
                    if (externalWall != null)
                    {
                        // Для режима "по стенам" тоже держим угол открытым
                        DisableWallJoins(externalWall);
                        CopyWallProperties(cand.InnerWall, externalWall);
                        created++;
                    }
                }
            }
            catch { }

            return created;
        }

        /// <summary>
        /// Create external walls for a set of existing walls, dividing them into segments based on the number of adjacent rooms.
        /// </summary>
        public static int CreateSegmentedExternalWalls(Document doc, List<Wall> walls, WallType wallType)
        {
            if (doc == null || walls == null || walls.Count == 0 || wallType == null)
                return 0;

            int created = 0;

            try
            {
                foreach (var innerWall in walls)
                {
                    if (innerWall == null)
                        continue;

                    if (!(innerWall.Location is LocationCurve lc) || lc.Curve == null)
                        continue;

                    Curve baseCurve = lc.Curve;
                    if (!(baseCurve is Line) || baseCurve.Length < 0.01)
                        continue; // Only process straight walls

                    // Get wall properties
                    double existingThickness = GetWallThickness(innerWall);
                    double newThickness = GetWallTypeThickness(wallType);
                    double totalOffsetDistance = (existingThickness / 2.0) + (newThickness / 2.0);

                    XYZ wallFaceNormal = GetWallFaceNormal(innerWall);
                    var offsetCurves = GeometryUtilities.OffsetCurve(baseCurve, totalOffsetDistance, wallFaceNormal);
                    if (offsetCurves == null || offsetCurves.Count == 0)
                        continue;

                    Curve offset = offsetCurves[0];
                    if (offset == null || offset.Length < 0.01)
                        continue;

                    // Divide the curve into segments based on the number of adjacent rooms
                    List<Curve> segments = DivideCurveByRooms(offset, innerWall);

                    foreach (var segment in segments)
                    {
                        if (segment == null || segment.Length < 0.01)
                            continue;

                        Level level = GetWallLevel(innerWall);
                        double height = GetWallHeight(innerWall);
                        if (level == null)
                            continue;

                        Wall externalWall = Wall.Create(doc, segment, wallType.Id, level.Id, height, 0.0, false, false);
                        if (externalWall != null)
                        {
                            DisableWallJoins(externalWall);
                            CopyWallProperties(innerWall, externalWall);
                            created++;
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                throw new Exception($"Error creating segmented external walls: {ex.Message}");
            }

            return created;
        }

        /// <summary>
        /// Divide a curve into segments based on the number of adjacent rooms.
        /// </summary>
        private static List<Curve> DivideCurveByRooms(Curve curve, Wall wall)
        {
            var segments = new List<Curve>();

            try
            {
                // Get adjacent rooms
                var adjacentRooms = GetAdjacentRooms(wall);
                if (adjacentRooms == null || adjacentRooms.Count == 0)
                {
                    segments.Add(curve);
                    return segments;
                }

                // Divide the curve into equal segments based on the number of rooms
                double segmentLength = curve.Length / adjacentRooms.Count;
                double currentLength = 0.0;

                while (currentLength < curve.Length)
                {
                    double nextLength = Math.Min(currentLength + segmentLength, curve.Length);

                    // Create a new segment using the curve's parameter space
                    double startParam = curve.Project(curve.GetEndPoint(0)).Parameter + 
                                        (currentLength / curve.Length) * 
                                        (curve.Project(curve.GetEndPoint(1)).Parameter - curve.Project(curve.GetEndPoint(0)).Parameter);
                    double endParam = curve.Project(curve.GetEndPoint(0)).Parameter + 
                                      (nextLength / curve.Length) * 
                                      (curve.Project(curve.GetEndPoint(1)).Parameter - curve.Project(curve.GetEndPoint(0)).Parameter);

                    Curve segment = curve.Clone();
                    segment.MakeBound(startParam, endParam);

                    if (segment != null && segment.Length > 0.01)
                    {
                        segments.Add(segment);
                    }

                    currentLength = nextLength;
                }
            }
            catch
            {
                // If division fails, return the original curve
                segments.Add(curve);
            }

            return segments;
        }

        /// <summary>
        /// Get the rooms adjacent to a wall.
        /// </summary>
        private static List<ElementId> GetAdjacentRooms(Wall wall)
        {
            var roomIds = new List<ElementId>();

            try
            {
                Document doc = wall.Document;
                var spatialElementBoundaryOptions = new SpatialElementBoundaryOptions();

                // Get the rooms in the document
                FilteredElementCollector collector = new FilteredElementCollector(doc).OfClass(typeof(SpatialElement));
                foreach (SpatialElement element in collector)
                {
                    if (element is Room room)
                    {
                        var boundaries = room.GetBoundarySegments(spatialElementBoundaryOptions);
                        if (boundaries != null)
                        {
                            foreach (var boundary in boundaries)
                            {
                                foreach (var segment in boundary)
                                {
                                    if (segment.ElementId == wall.Id)
                                    {
                                        roomIds.Add(room.Id);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            catch
            {
                // Ignore errors and return an empty list
            }

            return roomIds;
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
        /// <summary>
        /// Trim a candidate wall curve so it does not intersect provided walls.
        /// Использует толщину нашей и соседней стены в узле, чтобы дотягивать сегмент до внешнего угла.
        /// </summary>
        private static Curve TrimCurveAgainstExisting(Curve candidate, IEnumerable<WallCurveInfo> existingWalls, double primaryHalfThickness)
        {
            if (candidate == null || !(candidate is Line candLine))
                return candidate;

            if (existingWalls == null)
                return candidate;

            const double minLength = 0.5; // ~150 мм
            const double endTol = 1e-4;   // узел в торце

            XYZ start = candLine.GetEndPoint(0);
            XYZ end = candLine.GetEndPoint(1);
            XYZ dir = (end - start).Normalize();

            foreach (WallCurveInfo info in existingWalls)
            {
                if (!(info?.Curve is Line existing))
                    continue;

                try
                {
                    SetComparisonResult res = candLine.Intersect(existing, out IntersectionResultArray arr);
                    if (res == SetComparisonResult.Disjoint)
                        continue;

                    var hits = new List<XYZ>();
                    if (arr != null && !arr.IsEmpty)
                    {
                        for (int i = 0; i < arr.Size; i++)
                        {
                            XYZ p = arr.get_Item(i)?.XYZPoint;
                            if (p != null)
                                hits.Add(p);
                        }
                    }

                    if (hits.Count == 0 && (res == SetComparisonResult.Overlap ||
                                            res == SetComparisonResult.Subset ||
                                            res == SetComparisonResult.Superset))
                    {
                        hits.Add(existing.GetEndPoint(0));
                        hits.Add(existing.GetEndPoint(1));
                    }

                    XYZ exA = existing.GetEndPoint(0);
                    XYZ exB = existing.GetEndPoint(1);
                    XYZ exDir = (exB - exA).Normalize();
                    XYZ exNormal = new XYZ(-exDir.Y, exDir.X, 0.0);

                    foreach (var p in hits)
                    {
                        if (p == null)
                            continue;

                        // Ищем вторую стену, которая тоже приходит в этот узел
                        double otherHalf = 0.0;
                        foreach (var other in existingWalls)
                        {
                            if (other == info) continue;
                            if (!(other.Curve is Line otherLine)) continue;

                            XYZ oa = otherLine.GetEndPoint(0);
                            XYZ ob = otherLine.GetEndPoint(1);
                            if (p.DistanceTo(oa) < endTol || p.DistanceTo(ob) < endTol)
                            {
                                double th = GetWallThickness(other.Wall) / 2.0;
                                if (th > otherHalf) otherHalf = th;
                            }
                        }

                        double totalHalf = primaryHalfThickness + otherHalf;

                        if (totalHalf < 1e-6)
                        {
                            // нет данных по толщинам – режем по точке пересечения
                            double d0 = p.DistanceTo(start);
                            double d1 = p.DistanceTo(end);
                            if (d0 <= d1) start = p; else end = p;

                            if (start.DistanceTo(end) < minLength)
                                return null;

                            candLine = Line.CreateBound(start, end);
                            dir = (end - start).Normalize();
                            continue;
                        }

                        double dHit = (p - exA).DotProduct(exNormal);
                        double target = (dHit >= 0 ? 1.0 : -1.0) * totalHalf;

                        double distStart = p.DistanceTo(start);
                        double distEnd = p.DistanceTo(end);
                        bool moveStart = distStart <= distEnd;

                        XYZ basePt = moveStart ? start : end;
                        double a0 = (basePt - exA).DotProduct(exNormal);
                        double denom = dir.DotProduct(exNormal);

                        if (Math.Abs(denom) < 1e-9)
                        {
                            if (moveStart) start = p; else end = p;
                        }
                        else
                        {
                            double t = (target - a0) / denom;
                            XYZ newPt = basePt + dir * t;
                            if (moveStart) start = newPt; else end = newPt;
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
        /// (В данный момент для сценария "по комнатам" не используется — там
        /// замыкание углов выполняем отдельной функцией на наборе кандидатов.)
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

                        // Дальше — та же логика, что в TrimCurveAgainstExisting,
                        // только без повторного вычисления нормали и толщины.
                        // В том числе, если пересечение попадает в торец одной из стен,
                        // мы всё равно «подтягиваем» вторую к этой общей точке,
                        // чтобы стены сходились точно в вершине угла без взаимного захода.
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
        /// Жёстко «склеивает» внешние стены в вершинах углов:
        /// для каждой пары пересекающихся кандидатов сдвигает ближайший к пересечению торец
        /// РЕГУЛЯРНО для ОБОИХ сегментов, чтобы они оба заканчивались ровно в точке пересечения
        /// без наложения и без зазора.
        /// Работает только с линейными кривыми (Line).
        /// </summary>
        private static void AdjustExternalCandidatesAtIntersections(List<ExternalWallCandidate> candidates, double externalHalfThickness)
        {
            if (candidates == null || candidates.Count < 2)
                return;

            const double minLength = 0.5; // ~150 мм

            // Локальный массив, чтобы удобнее править кривые по индексу
            Curve[] curves = candidates.Select(c => c?.Curve).ToArray();

            // Локальная функция пересечения БЕСКОНЕЧНЫХ прямых в XY,
            // даже если отрезки пока не соприкасаются.
            XYZ IntersectInfiniteLines2D(Line a, Line b)
            {
                XYZ p1 = a.GetEndPoint(0);
                XYZ p2 = b.GetEndPoint(0);
                XYZ v1 = a.GetEndPoint(1) - p1;
                XYZ v2 = b.GetEndPoint(1) - p2;

                double det = v1.X * v2.Y - v1.Y * v2.X;
                if (Math.Abs(det) < 1e-9)
                    return null; // параллельные или почти параллельные

                double t = ((p2.X - p1.X) * v2.Y - (p2.Y - p1.Y) * v2.X) / det;
                XYZ p = p1 + v1 * t;
                // Z берём из первой линии, чтобы остаться в плоскости уровня
                return new XYZ(p.X, p.Y, p1.Z);
            }

            for (int i = 0; i < curves.Length; ++i)
            {
                if (!(curves[i] is Line))
                    continue;

                for (int j = i + 1; j < curves.Length; ++j)
                {
                    if (!(curves[j] is Line))
                        continue;

                    Line li = curves[i] as Line;
                    Line lj = curves[j] as Line;
                    if (li == null || lj == null)
                        continue;

                    // Параллельные/коллинеарные сегменты трогать не нужно — они и так лежат в одной линии.
                    XYZ di = (li.GetEndPoint(1) - li.GetEndPoint(0)).Normalize();
                    XYZ dj = (lj.GetEndPoint(1) - lj.GetEndPoint(0)).Normalize();
                    if (di.CrossProduct(dj).GetLength() < 1e-6)
                        continue;

                    try
                    {
                        // Пытаемся найти точку пересечения:
                        // 1) сначала обычное пересечение отрезков,
                        // 2) если отрезки пока не пересекаются (Disjoint) — пересечение бесконечных линий.
                        XYZ p = null;
                        SetComparisonResult res = li.Intersect(lj, out IntersectionResultArray arr);
                        if (res != SetComparisonResult.Disjoint && arr != null && !arr.IsEmpty)
                        {
                            p = arr.get_Item(0)?.XYZPoint;
                        }
                        else if (res == SetComparisonResult.Disjoint)
                        {
                            p = IntersectInfiniteLines2D(li, lj);
                        }

                        if (p == null)
                            continue;

                        // Определяем, с каким типом угла имеем дело.
                        // Нас интересуют только ВНЕШНИЕ (выпуклые) углы здания.
                        // Для этого проверяем, что точка пересечения лежит с "наружной"
                        // стороны обеих исходных стен (по нормали GetWallFaceNormal).
                        bool isConvexForI = false;
                        bool isConvexForJ = false;
                        try
                        {
                            var locI = candidates[i].InnerWall.Location as LocationCurve;
                            var locJ = candidates[j].InnerWall.Location as LocationCurve;
                            if (locI?.Curve is Line lineI && locJ?.Curve is Line lineJ)
                            {
                                XYZ wi = lineI.GetEndPoint(0);
                                XYZ wj = lineJ.GetEndPoint(0);
                                XYZ nI = GetWallFaceNormal(candidates[i].InnerWall);
                                XYZ nJ = GetWallFaceNormal(candidates[j].InnerWall);

                                double sideI = (p - wi).DotProduct(nI);
                                double sideJ = (p - wj).DotProduct(nJ);

                                // Если точка пересечения лежит по наружной нормали
                                // для обеих стен, считаем угол внешним.
                                isConvexForI = sideI > 0;
                                isConvexForJ = sideJ > 0;
                            }
                        }
                        catch { }

                        // Внутренние (вогнутые) углы, например в "Г"-образной комнате,
                        // пропускаем, чтобы не стягивать там внешние стены и не ломать
                        // внутреннюю геометрию.
                        if (!isConvexForI || !isConvexForJ)
                            continue;

                        // Жёстко корректируем оба сегмента около точки пересечения осей.
                        // Но чтобы новые стены НЕ заходили друг в друга, а доходили только
                        // до внешнего угла, укорачиваем ось на половину толщины создаваемой
                        // стены от точки пересечения вдоль направления сегмента.
                        XYZ si = li.GetEndPoint(0);
                        XYZ ei = li.GetEndPoint(1);
                        double dsi = si.DistanceTo(p);
                        double dei = ei.DistanceTo(p);
                        {
                            bool siCloser = dsi <= dei;
                            XYZ near = siCloser ? si : ei;
                            XYZ far = siCloser ? ei : si;

                            // Вектор от ближайшего торца к точке пересечения осей
                            XYZ vToP = p - near;
                            double lenToP = vToP.GetLength();
                            if (lenToP > externalHalfThickness + 1e-6)
                            {
                                XYZ dirToP = vToP.Normalize();
                                // Сдвигаем конец ОТ точки пересечения внутрь сегмента
                                // на величину externalHalfThickness
                                XYZ newEnd = near + dirToP * (lenToP - externalHalfThickness);
                                XYZ newOther = far;

                                if (newEnd.DistanceTo(newOther) >= minLength)
                                {
                                    li = siCloser
                                        ? Line.CreateBound(newEnd, newOther)
                                        : Line.CreateBound(newOther, newEnd);
                                }
                            }
                        }

                        XYZ sj = lj.GetEndPoint(0);
                        XYZ ej = lj.GetEndPoint(1);
                        double dsj = sj.DistanceTo(p);
                        double dej = ej.DistanceTo(p);
                        {
                            bool sjCloser = dsj <= dej;
                            XYZ near = sjCloser ? sj : ej;
                            XYZ far = sjCloser ? ej : sj;

                            XYZ vToP = p - near;
                            double lenToP = vToP.GetLength();
                            if (lenToP > externalHalfThickness + 1e-6)
                            {
                                XYZ dirToP = vToP.Normalize();
                                XYZ newEnd = near + dirToP * (lenToP - externalHalfThickness);
                                XYZ newOther = far;

                                if (newEnd.DistanceTo(newOther) >= minLength)
                                {
                                    lj = sjCloser
                                        ? Line.CreateBound(newEnd, newOther)
                                        : Line.CreateBound(newOther, newEnd);
                                }
                            }
                        }

                        curves[i] = li;
                        curves[j] = lj;
                    }
                    catch
                    {
                        // игнорируем ошибки пересечения и двигаемся дальше
                    }
                }
            }

            // Записываем обновлённые кривые обратно в кандидатов
            for (int k = 0; k < candidates.Count && k < curves.Length; ++k)
            {
                if (curves[k] != null)
                    candidates[k].Curve = curves[k];
            }
        }

        /// <summary>
        /// Collect straight location curves from existing walls in the document,
        /// optionally excluding a specific wall (e.g., the source wall).
        /// </summary>
        private static List<WallCurveInfo> GetExistingWallCurves(Document doc, Wall excludeWall = null)
        {
            var infos = new List<WallCurveInfo>();
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
                        infos.Add(new WallCurveInfo
                        {
                            Wall = w,
                            Curve = line
                        });
                    }
                }
            }
            catch { }

            return infos;
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
            // Сначала считаем ВСЕ геометрические кандидаты, потом жёстко стягиваем их в углах,
            // и только после этого создаём реальные стены. Так мы исключаем влияние порядка обхода.
            var candidates = new List<ExternalWallCandidate>();

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

                // Для каждой исходной стены получаем сегменты по комнатам
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

                    if (wallSegments == null || wallSegments.Count == 0)
                        continue;

                    // Для каждого сегмента сразу считаем его внешнюю (offset) линию,
                    // но реальные стены пока НЕ создаём, только накапливаем кандидатов.
                    double existingThickness = GetWallThickness(innerWall);
                    double newThickness = GetWallTypeThickness(wallType);
                    double totalOffsetDistance = (existingThickness / 2.0) + (newThickness / 2.0);
                    XYZ wallFaceNormal = GetWallFaceNormal(innerWall);

                    foreach (Curve segment in wallSegments)
                    {
                        if (segment == null || segment.Length < 0.01)
                            continue;

                        var offsetCurves = GeometryUtilities.OffsetCurve(segment, totalOffsetDistance, wallFaceNormal);
                        if (offsetCurves == null || offsetCurves.Count == 0)
                            continue;

                        Curve offset = offsetCurves[0];
                        if (offset == null || offset.Length < 0.01)
                            continue;

                        // Разворачиваем, чтобы внутренняя грань новой стены смотрела на исходную.
                        Curve reversed = offset.CreateReversed();

                        candidates.Add(new ExternalWallCandidate
                        {
                            InnerWall = innerWall,
                            Curve = reversed
                        });
                    }
                }

                // Жёстко стягиваем все кандидаты в вершинах углов (на уровне геометрии),
                // учитывая половину толщины создаваемой внешней стены, чтобы они
                // доходили до угла, но не заходили друг в друга.
                double externalHalfThickness = GetWallTypeThickness(wallType) / 2.0;
                AdjustExternalCandidatesAtIntersections(candidates, externalHalfThickness);

                // Теперь создаём реальные стены по уже скорректированным кривым.
                foreach (var cand in candidates)
                {
                    if (cand?.Curve == null || cand.Curve.Length < 0.01 || cand.InnerWall == null)
                        continue;

                    Level level = GetWallLevel(cand.InnerWall);
                    double height = GetWallHeight(cand.InnerWall);
                    if (level == null)
                        continue;

                    Wall externalWall = Wall.Create(doc, cand.Curve, wallType.Id, level.Id, height, 0.0, false, false);
                    if (externalWall != null)
                    {
                        // В этом сценарии явно отключаем join'ы, чтобы угол оставался
                        // геометрически «открытым» и Revit не достраивал дополнительный угол.
                        DisableWallJoins(externalWall);
                        CopyWallProperties(cand.InnerWall, externalWall);
                        created++;
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
                List<WallCurveInfo> existingWallCurves = GetExistingWallCurves(doc, innerWall);
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
        private static Wall CreateExternalWallAlongCurveSingle(Document doc, Wall innerWall, Curve innerCurve, WallType wallType, List<Curve> existingExternalCurves = null, List<WallCurveInfo> existingWallCurves = null)
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
                double totalOffsetDistance = (existingThickness / 2.0) + (newThickness / 2.0);

                XYZ wallFaceNormal = GetWallFaceNormal(innerWall);

                List<Curve> offsetCurves = GeometryUtilities.OffsetCurve(innerCurve, totalOffsetDistance, wallFaceNormal);
                if (offsetCurves.Count == 0 || offsetCurves[0] == null)
                    return null;

                Curve offsetCurve = offsetCurves[0];
                Curve trimmed = offsetCurve.CreateReversed();

                // 1) Для сценария "внешние стены по комнатам" не режем кривую по существующим
                // внутренним стенам, чтобы не укорачивать сегменты на внешних углах.
                // Логику подрезки по существующим стенам оставляем для других сценариев
                // в отдельном методе CreateExternalWallAlongCurve.

                // 2) trim against already created external walls
                if (existingExternalCurves != null && existingExternalCurves.Count > 0)
                {
                    // Для внешних стен в узле нам важно, чтобы они сходились точно в точке пересечения,
                    // а не останавливались на расстоянии половины толщины.
                    // Поэтому при обрезке по уже созданным внешним стенам не задаём дополнительный offset.
                    trimmed = TrimCurveAgainstExternalCurves(trimmed, existingExternalCurves, 0.0);
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
        /// Create a single external wall along the curve specifically for the
        /// "CreateExternalWallsFromRooms" scenario.
        /// Здесь:
        /// - не подрезаем по существующим/созданным стенам;
        /// - не отключаем join'ы, чтобы Revit сам замыкал углы в точке пересечения
        ///   внешних стен, без зазоров и наложений.
        /// </summary>
        private static Wall CreateExternalWallAlongCurveForRooms(Document doc, Wall innerWall, Curve innerCurve, WallType wallType)
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
                double totalOffsetDistance = (existingThickness / 2.0) + (newThickness / 2.0);

                XYZ wallFaceNormal = GetWallFaceNormal(innerWall);

                List<Curve> offsetCurves = GeometryUtilities.OffsetCurve(innerCurve, totalOffsetDistance, wallFaceNormal);
                if (offsetCurves == null || offsetCurves.Count == 0)
                    return null;

                Curve offsetCurve = offsetCurves[0];
                if (offsetCurve == null || offsetCurve.Length < 0.01)
                    return null;

                Curve reversed = offsetCurve.CreateReversed();

                Wall externalWall = Wall.Create(doc, reversed, wallType.Id, level.Id, height, 0.0, false, false);
                if (externalWall != null)
                {
                    // Не вызываем DisableWallJoins — позволяем Revit автоматически
                    // оформить пересечения в углах.
                    CopyWallProperties(innerWall, externalWall);
                }

                return externalWall;
            }
            catch
            {
                return null;
            }
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

                    // Здесь мы делим строго по границам комнат (min/max + середины между ними).
                    // Внешние углы теперь корректируются отдельной функцией
                    // AdjustExternalCandidatesAtIntersections, поэтому дополнительно
                    // "вытягивать" крайние сегменты до концов несущей стены не нужно.
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
