using Autodesk.Revit.DB;
using Autodesk.Revit.DB.Architecture;
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
        /// Helper to write debug messages to Revit journal instead of Console.
        /// Это гарантированно попадает в journal*.txt, который вы потом копируете в README.
        /// </summary>
        private static void Log(Document doc, string message)
        {
            try
            {
                if (doc != null)
                {
                    // Пишем в журнал приложения Revit
                    doc.Application.WriteJournalComment("[AutoExternalWall] " + message, false);
                }
                else
                {
                    // На всякий случай – в отладочный вывод, если документа нет
                    System.Diagnostics.Debug.WriteLine("[AutoExternalWall] " + message);
                }
            }
            catch
            {
                // Никогда не ломаем основную логику из‑за проблем с логированием
            }
        }

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

                    reversedCurve = ExtendToWallEnds(innerWall, reversedCurve);


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
        /// Divide a curve into segments based on the boundaries of adjacent rooms.
        /// </summary>
        private static List<Curve> DivideCurveByRooms(Curve curve, Wall wall)
        {
            var segments = new List<Curve>();

            try
            {
                // Get adjacent rooms
                var adjacentRooms = GetAdjacentRooms(wall);
                Console.WriteLine($"Wall ID: {wall.Id}, Adjacent Rooms Count: {adjacentRooms.Count}");

                if (adjacentRooms == null || adjacentRooms.Count == 0)
                {
                    segments.Add(curve);
                    return segments;
                }

                Document doc = wall.Document;
                var spatialElementBoundaryOptions = new SpatialElementBoundaryOptions();

                // Iterate through each room and get its boundaries
                foreach (var roomId in adjacentRooms)
                {
                    Room room = doc.GetElement(roomId) as Room;
                    if (room == null)
                        continue;

                    var boundaries = room.GetBoundarySegments(spatialElementBoundaryOptions);
                    if (boundaries == null)
                        continue;

                    foreach (var boundary in boundaries)
                    {
                        foreach (var segment in boundary)
                        {
                            Curve boundaryCurve = segment.GetCurve();
                            if (boundaryCurve == null)
                                continue;

                            // Check intersection between the wall curve and the room boundary curve
                            IntersectionResultArray results;
                            SetComparisonResult result = curve.Intersect(boundaryCurve, out results);

                            if (result == SetComparisonResult.Overlap && results != null && results.Size > 0)
                            {
                                foreach (IntersectionResult intersection in results)
                                {
                                    XYZ intersectionPoint = intersection.XYZPoint;

                                    // Split the wall curve at the intersection point
                                    double param = curve.Project(intersectionPoint).Parameter;
                                    Curve segmentCurve = curve.Clone();
                                    segmentCurve.MakeBound(curve.GetEndParameter(0), param);

                                    if (segmentCurve.Length > 0.01)
                                    {
                                        Console.WriteLine($"Segment Created: Start = {segmentCurve.GetEndPoint(0)}, End = {segmentCurve.GetEndPoint(1)}, Length = {segmentCurve.Length}");
                                        segments.Add(segmentCurve);
                                    }

                                    // Update the curve to start from the intersection point
                                    curve.MakeBound(param, curve.GetEndParameter(1));
                                }
                            }
                        }
                    }
                }

                // Add the remaining part of the curve if it exists
                if (curve.Length > 0.01)
                {
                    segments.Add(curve);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error in DivideCurveByRooms: {ex.Message}");
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

                Console.WriteLine($"GetAdjacentRooms: Found {roomIds.Count} rooms for Wall ID {wall.Id}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error in GetAdjacentRooms: {ex.Message}");
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
        /// Calculate outward normal for a room boundary segment relative to a wall.
        /// If the room lies on the side of the wall normal, we flip the normal so that
        /// the returned vector always points "наружу" от помещения.
        /// </summary>
        private static XYZ GetOutwardNormalForRoomBoundary(Wall wall, Curve boundaryCurve)
        {
            XYZ wallNormal = GetWallFaceNormal(wall);
            if (wall == null || boundaryCurve == null)
                return wallNormal;

            try
            {
                if (!(wall.Location is LocationCurve lc) || !(lc.Curve is Line wallLine))
                    return wallNormal;

                // Берём середину граничной кривой помещения
                XYZ mid = boundaryCurve.Evaluate(0.5, true);
                // Находим ближайшую точку оси стены
                XYZ proj = wallLine.Project(mid)?.XYZPoint ?? wallLine.Evaluate(0.5, true);
                XYZ toRoom = mid - proj;

                // Если помещение лежит по направлению wallNormal, то наружу — в обратную сторону
                if (toRoom.DotProduct(wallNormal) > 0)
                    return wallNormal.Negate();

                return wallNormal;
            }
            catch
            {
                return wallNormal;
            }
        }

        /// <summary>
        /// Определяет направление наружу от помещения, используя центр помещения.
        /// Возвращает нормаль, направленную от помещения наружу.
        /// </summary>
        private static XYZ GetOutwardNormalFromRoom(Wall wall, Curve boundaryCurve, Room room)
        {
            XYZ wallNormal = GetWallFaceNormal(wall);
            if (wall == null || boundaryCurve == null || room == null)
                return wallNormal;

            try
            {
                if (!(wall.Location is LocationCurve lc) || !(lc.Curve is Line wallLine))
                    return wallNormal;

                // Получаем центр помещения
                LocationPoint roomLocation = room.Location as LocationPoint;
                if (roomLocation == null)
                {
                    // Если нет LocationPoint, используем граничную кривую
                    return GetOutwardNormalForRoomBoundary(wall, boundaryCurve);
                }

                XYZ roomCenter = roomLocation.Point;
                
                // Находим ближайшую точку на оси стены к центру помещения
                XYZ closestPointOnWall = wallLine.Project(roomCenter)?.XYZPoint;
                if (closestPointOnWall == null)
                {
                    // Если проекция не удалась, используем середину стены
                    closestPointOnWall = wallLine.Evaluate(0.5, true);
                }

                // Вектор от стены к центру помещения
                XYZ fromWallToRoom = (roomCenter - closestPointOnWall).Normalize();

                // Если нормаль стены направлена в сторону помещения, инвертируем её
                // чтобы получить направление наружу от помещения
                if (wallNormal.DotProduct(fromWallToRoom) > 0)
                {
                    return -wallNormal;
                }

                return wallNormal;
            }
            catch
            {
                // В случае ошибки используем базовую функцию
                return GetOutwardNormalForRoomBoundary(wall, boundaryCurve);
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


private static Curve ExtendToWallEnds(Wall sourceWall, Curve curve)
{
    if (!(curve is Line line))
        return curve;

    if (!(sourceWall.Location is LocationCurve lc))
        return curve;

    if (!(lc.Curve is Line axis))
        return curve;

    XYZ dir = (axis.GetEndPoint(1) - axis.GetEndPoint(0)).Normalize();

    double halfWidth = GetWallThickness(sourceWall) / 2.0;

    XYZ p0 = line.GetEndPoint(0) - dir * halfWidth;
    XYZ p1 = line.GetEndPoint(1) + dir * halfWidth;

    return Line.CreateBound(p0, p1);
}

        private static double GetJoinExtensionLength(
    Wall wall,
    int endIndex // 0 или 1
)
{
    var joined = JoinGeometryUtils.GetJoinedElements(
        wall.Document,
        wall
    );

    foreach (ElementId id in joined)
    {
        if (!(wall.Document.GetElement(id) is Wall other))
            continue;

        // Updated to provide the secondElement parameter
        if (!JoinGeometryUtils.AreElementsJoined(wall.Document, wall, other))
            continue;

        // Updated logic to check join condition
        if (!JoinGeometryUtils.IsCuttingElementInJoin(wall.Document, wall, other))
            continue;

        // толщина второй стены
        return GetWallThickness(other) / 2.0;
    }

    return 0.0;
}
private static Curve ExtendCurveToJoinedWalls(
    Wall sourceWall,
    Curve curve
)
{
    if (!(sourceWall.Location is LocationCurve lc))
        return curve;

    if (!(lc.Curve is Line axis))
        return curve;

    XYZ a = axis.GetEndPoint(0);
    XYZ b = axis.GetEndPoint(1);
    XYZ dir = (b - a).Normalize();

    XYZ p0 = curve.GetEndPoint(0);
    XYZ p1 = curve.GetEndPoint(1);

    double ext0 = GetJoinExtensionLength(sourceWall, 0);
    double ext1 = GetJoinExtensionLength(sourceWall, 1);

    XYZ newP0 = p0 - dir * ext0;
    XYZ newP1 = p1 + dir * ext1;

    return Line.CreateBound(newP0, newP1);
}

        /// <summary>
        /// Новая простая логика создания внешних стен для одного помещения.
        /// Создает внешние стены, повторяющие границы стен помещения.
        /// </summary>
        public static int CreateExternalWallsFromSingleRoom(Document doc, Room room, WallType wallType)
        {
            if (doc == null || room == null || wallType == null)
                return 0;

            int created = 0;

            try
            {
                Log(doc, $"Начинаем создание внешних стен для помещения {room.Id}");

                // Получаем границы помещения
                SpatialElementBoundaryOptions options = new SpatialElementBoundaryOptions();
                IList<IList<BoundarySegment>> boundaryLoops = room.GetBoundarySegments(options);

                if (boundaryLoops == null || boundaryLoops.Count == 0)
                {
                    Log(doc, $"Помещение {room.Id} не имеет границ");
                    return 0;
                }

                Log(doc, $"Найдено граничных контуров: {boundaryLoops.Count}");

                // Собираем все стены из границ помещения
                List<Wall> roomWalls = new List<Wall>();
                Dictionary<ElementId, Curve> wallBoundaryCurves = new Dictionary<ElementId, Curve>();

                foreach (IList<BoundarySegment> loop in boundaryLoops)
                {
                    foreach (BoundarySegment segment in loop)
                    {
                        Element boundaryElement = doc.GetElement(segment.ElementId);
                        if (boundaryElement is Wall wall)
                        {
                            // Получаем кривую границы помещения для этой стены
                            Curve boundaryCurve = segment.GetCurve();
                            if (boundaryCurve != null)
                            {
                                if (!roomWalls.Contains(wall))
                                {
                                    roomWalls.Add(wall);
                                }
                                // Сохраняем кривую границы для стены
                                wallBoundaryCurves[wall.Id] = boundaryCurve;
                            }
                        }
                    }
                }

                Log(doc, $"Найдено стен в границах помещения: {roomWalls.Count}");

                if (roomWalls.Count == 0)
                {
                    Log(doc, "Нет стен в границах помещения");
                    return 0;
                }

                // Для каждой стены создаем внешнюю стену по её внешней длине
                foreach (Wall innerWall in roomWalls)
                {
                    // Получаем центральную линию существующей стены
                    LocationCurve wallLocation = innerWall.Location as LocationCurve;
                    if (wallLocation == null || wallLocation.Curve == null)
                    {
                        Log(doc, $"Стена {innerWall.Id} не имеет LocationCurve");
                        continue;
                    }

                    Curve wallCenterLine = wallLocation.Curve;
                    if (wallCenterLine == null || wallCenterLine.Length < 0.01)
                    {
                        Log(doc, $"Стена {innerWall.Id} имеет слишком короткую центральную линию");
                        continue;
                    }

                    Log(doc, $"Обрабатываем стену {innerWall.Id}, длина центральной линии: {wallCenterLine.Length:F3}");

                    // Получаем параметры стены
                    Level level = GetWallLevel(innerWall);
                    double height = GetWallHeight(innerWall);
                    if (level == null)
                    {
                        Log(doc, $"Не удалось получить уровень для стены {innerWall.Id}");
                        continue;
                    }

                    // Вычисляем смещение от центральной линии внутренней стены к центральной линии внешней стены
                    // Смещение = половина толщины внутренней стены + половина толщины внешней стены
                    double innerThickness = GetWallThickness(innerWall);
                    double externalThickness = GetWallTypeThickness(wallType);
                    double offsetDistance = (innerThickness) + (externalThickness);

                    Log(doc, $"Смещение: {offsetDistance:F3} (толщина внутренней: {innerThickness:F3}, внешней: {externalThickness:F3})");

                    // Определяем направление наружу от помещения
                    // Используем граничную кривую для определения направления
                    Curve boundaryCurve = null;
                    if (wallBoundaryCurves.TryGetValue(innerWall.Id, out boundaryCurve))
                    {
                        // Используем граничную кривую только для определения направления
                    }
                    
                    XYZ outwardNormal = GetOutwardNormalFromRoom(innerWall, boundaryCurve ?? wallCenterLine, room);

                    // Смещаем центральную линию стены наружу
                    List<Curve> offsetCurves = GeometryUtilities.OffsetCurve(wallCenterLine, offsetDistance, outwardNormal);
                    
                    if (offsetCurves == null || offsetCurves.Count == 0)
                    {
                        Log(doc, $"Не удалось сместить кривую для стены {innerWall.Id}");
                        continue;
                    }

                    // Берем первую смещенную кривую и разворачиваем её (чтобы внутренняя грань была обращена к исходной стене)
                    Curve externalCurve = offsetCurves[0].CreateReversed();

                    if (externalCurve == null || externalCurve.Length < 0.01)
                    {
                        Log(doc, $"Смещенная кривая слишком короткая для стены {innerWall.Id}");
                        continue;
                    }

                    Log(doc, $"Создаем внешнюю стену по кривой длиной {externalCurve.Length:F3}");

                    // Создаем внешнюю стену
                    Wall externalWall = Wall.Create(
                        doc,
                        externalCurve,
                        wallType.Id,
                        level.Id,
                        height,
                        0.0,
                        false,
                        false
                    );

                    if (externalWall != null)
                    {
                        // Отключаем соединения стен, чтобы углы оставались открытыми
                        DisableWallJoins(externalWall);
                        // Копируем свойства из внутренней стены
                        CopyWallProperties(innerWall, externalWall);
                        created++;
                        Log(doc, $"Создана внешняя стена {externalWall.Id} для внутренней стены {innerWall.Id}");
                    }
                    else
                    {
                        Log(doc, $"Не удалось создать внешнюю стену для {innerWall.Id}");
                    }
                }

                Log(doc, $"Всего создано внешних стен: {created}");
            }
            catch (Exception ex)
            {
                Log(doc, $"Ошибка при создании внешних стен: {ex.Message}");
                throw new Exception($"Error creating external walls from room: {ex.Message}", ex);
            }

            return created;
        }

    }
}
