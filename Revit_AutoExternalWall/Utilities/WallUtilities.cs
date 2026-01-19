using Autodesk.Revit.DB;
using Autodesk.Revit.DB.Architecture;
using System;
using System.Collections.Generic;
using System.Linq;
using Revit_AutoExternalWall;

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
        /// First checks settings for a saved wall type, then falls back to automatic selection
        /// </summary>
        public static WallType GetExternalWallType(Document doc)
        {
            try
            {
                // Сначала проверяем настройки на наличие сохраненного типа стены
                Settings settings = Settings.Load();
                if (!string.IsNullOrEmpty(settings.SelectedWallTypeId))
                {
                    if (int.TryParse(settings.SelectedWallTypeId, out int wallTypeIdInt))
                    {
                        ElementId wallTypeId = new ElementId(wallTypeIdInt);
                        WallType savedWallType = doc.GetElement(wallTypeId) as WallType;
                        if (savedWallType != null && savedWallType.Kind == WallKind.Basic)
                        {
                            return savedWallType;
                        }
                    }
                }

                // Если сохраненного типа нет или он не найден, используем автоматический поиск
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
                        
                        // Устанавливаем параметр ADSK_Зона для внешней стены
                        SetZoneParameter(doc, externalWall, null);
                        
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
        /// Проверяет, пересекает ли создаваемая стена границы помещения.
        /// Если пересекает - значит стена находится внутри комнаты и нужно инвертировать направление.
        /// </summary>
        private static bool DoesWallIntersectRoom(Curve externalCurve, Wall existingWall, Room room, Curve boundaryCurve = null)
        {
            if (externalCurve == null || existingWall == null || room == null)
                return false;

            try
            {
                // Получаем все границы помещения
                SpatialElementBoundaryOptions opt = new SpatialElementBoundaryOptions();
                IList<IList<BoundarySegment>> boundaryLoops = room.GetBoundarySegments(opt);
                
                if (boundaryLoops == null || boundaryLoops.Count == 0)
                    return false;

                // Проверяем пересечение создаваемой стены со всеми границами помещения
                // Исключаем boundary segment самой существующей стены
                foreach (IList<BoundarySegment> loop in boundaryLoops)
                {
                    foreach (BoundarySegment segment in loop)
                    {
                        // Пропускаем boundary segment существующей стены
                        if (segment.ElementId == existingWall.Id)
                            continue;

                        Curve boundarySegCurve = segment.GetCurve();
                        if (boundarySegCurve == null)
                            continue;

                        // Проверяем пересечение создаваемой стены с boundary segment
                        IntersectionResultArray intersectionResults;
                        SetComparisonResult intersection = externalCurve.Intersect(boundarySegCurve, out intersectionResults);

                        if (intersection != SetComparisonResult.Disjoint && 
                            intersectionResults != null && 
                            intersectionResults.Size > 0)
                        {
                            // Если есть пересечение с границей комнаты (кроме самой существующей стены),
                            // значит создаваемая стена пересекает комнату
                            return true;
                        }
                    }
                }

                return false;
            }
            catch
            {
                return false;
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
        /// Возвращает плановую (XY) кривую внешней грани стены.
        /// Используется, чтобы длина создаваемой стены соответствовала реальной внешней грани,
        /// а не осевой линии.
        /// </summary>
        private static Curve GetExteriorFacePlanCurve(Wall wall)
        {
            if (wall == null) return null;
            try
            {
                var sideRefs = HostObjectUtils.GetSideFaces(wall, ShellLayerType.Exterior);
                if (sideRefs == null || sideRefs.Count == 0)
                    return null;

                foreach (Reference r in sideRefs)
                {
                    Face face = wall.Document.GetElement(r)?.GetGeometryObjectFromReference(r) as Face;
                    if (face == null) continue;

                    foreach (EdgeArray ea in face.EdgeLoops)
                    {
                        foreach (Edge e in ea)
                        {
                            Curve c = e.AsCurve();
                            if (c == null || !c.IsBound) continue;

                            // Ищем горизонтальную (в плане) грань
                            if (Math.Abs(c.GetEndPoint(0).Z - c.GetEndPoint(1).Z) < 1e-6)
                            {
                                return c;
                            }
                        }
                    }
                }
            }
            catch { }

            return null;
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
                        
                        // Устанавливаем параметр ADSK_Зона для внешней стены
                        SetZoneParameter(doc, externalWall, null);
                        
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
                    
                    // Устанавливаем параметр ADSK_Зона для внешней стены
                    SetZoneParameter(doc, externalWall, null);
                    
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
                    
                    // Устанавливаем параметр ADSK_Зона для внешней стены
                    SetZoneParameter(doc, externalWall, null);
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

    XYZ axisStart = axis.GetEndPoint(0);
    XYZ axisEnd = axis.GetEndPoint(1);
    XYZ dir = (axisEnd - axisStart).Normalize();
    double halfWidth = GetWallThickness(sourceWall) / 2.0;

    // Находим примыкающие стены в углах
    Wall adjacentWall0 = FindAdjacentWallAtEnd(sourceWall, 0);
    Wall adjacentWall1 = FindAdjacentWallAtEnd(sourceWall, 1);

    XYZ p0 = line.GetEndPoint(0);
    XYZ p1 = line.GetEndPoint(1);

    // Продлеваем начало с учетом угла
    if (adjacentWall0 != null)
    {
        double extension0 = CalculateExtensionLength(sourceWall, adjacentWall0, 0);
        p0 = p0 - dir * extension0;
    }
    else
    {
        p0 = p0 - dir * halfWidth;
    }

    // Продлеваем конец с учетом угла
    if (adjacentWall1 != null)
    {
        double extension1 = CalculateExtensionLength(sourceWall, adjacentWall1, 1);
        p1 = p1 + dir * extension1;
    }
    else
    {
        p1 = p1 + dir * halfWidth;
    }

    return Line.CreateBound(p0, p1);
        }

        /// <summary>
/// Находит примыкающую стену в указанном конце исходной стены
        /// </summary>
private static Wall FindAdjacentWallAtEnd(Wall sourceWall, int endIndex)
{
    if (sourceWall == null || sourceWall.Document == null)
        return null;

    try
    {
        LocationCurve sourceLocation = sourceWall.Location as LocationCurve;
        if (sourceLocation == null || !(sourceLocation.Curve is Line sourceAxis))
            return null;

        XYZ sourceEndPoint = (endIndex == 0) ? sourceAxis.GetEndPoint(0) : sourceAxis.GetEndPoint(1);
        XYZ sourceDir = (sourceAxis.GetEndPoint(1) - sourceAxis.GetEndPoint(0)).Normalize();
        if (endIndex == 0) sourceDir = -sourceDir; // Для начала берем обратное направление

        // Ищем примыкающие стены через JoinGeometryUtils
        var joined = JoinGeometryUtils.GetJoinedElements(sourceWall.Document, sourceWall);
        if (joined == null)
            return null;

        double minDistance = double.MaxValue;
        Wall closestWall = null;

        foreach (ElementId id in joined)
        {
            if (!(sourceWall.Document.GetElement(id) is Wall other))
                continue;

            if (!JoinGeometryUtils.AreElementsJoined(sourceWall.Document, sourceWall, other))
                continue;

            LocationCurve otherLocation = other.Location as LocationCurve;
            if (otherLocation == null || !(otherLocation.Curve is Line otherAxis))
                continue;

            // Проверяем, находится ли конец другой стены близко к концу исходной
            XYZ otherStart = otherAxis.GetEndPoint(0);
            XYZ otherEnd = otherAxis.GetEndPoint(1);
            
            double distToStart = sourceEndPoint.DistanceTo(otherStart);
            double distToEnd = sourceEndPoint.DistanceTo(otherEnd);

            double minDist = Math.Min(distToStart, distToEnd);
            const double tolerance = 0.1; // 10 см

            if (minDist < tolerance && minDist < minDistance)
            {
                minDistance = minDist;
                closestWall = other;
            }
        }

        return closestWall;
    }
    catch
    {
        return null;
    }
}

/// <summary>
/// Вычисляет длину продления с учетом угла между стенами
/// Находит точку пересечения линии продления с внешней гранью примыкающей стены
/// </summary>
private static double CalculateExtensionLength(Wall sourceWall, Wall adjacentWall, int endIndex)
{
    if (sourceWall == null || adjacentWall == null)
    {
        return GetWallThickness(sourceWall) / 2.0;
    }

    try
    {
        LocationCurve sourceLocation = sourceWall.Location as LocationCurve;
        LocationCurve adjacentLocation = adjacentWall.Location as LocationCurve;
        
        if (sourceLocation == null || !(sourceLocation.Curve is Line sourceAxis))
            return GetWallThickness(sourceWall) / 2.0;
        
        if (adjacentLocation == null || !(adjacentLocation.Curve is Line adjacentAxis))
            return GetWallThickness(sourceWall) / 2.0;

        XYZ sourceEndPoint = (endIndex == 0) ? sourceAxis.GetEndPoint(0) : sourceAxis.GetEndPoint(1);
        XYZ sourceDir = (sourceAxis.GetEndPoint(1) - sourceAxis.GetEndPoint(0)).Normalize();
        if (endIndex == 0) sourceDir = -sourceDir;

        // Определяем направление нормали к исходной стене (наружу)
        // Нужно получить правильное направление наружу от помещения
        XYZ sourceNormal = GetWallFaceNormal(sourceWall);
        
        // Получаем внешнюю грань примыкающей стены
        var sideRefs = HostObjectUtils.GetSideFaces(adjacentWall, ShellLayerType.Exterior);
        if (sideRefs == null || sideRefs.Count == 0)
        {
            return GetWallThickness(sourceWall) / 2.0;
        }

        // Строим линию продления: от конца оси исходной стены в направлении нормали
        // Эта линия должна пересечь внешнюю грань примыкающей стены
        Line extensionLine = Line.CreateUnbound(sourceEndPoint, sourceNormal);

        // Находим точку пересечения линии продления с внешней гранью примыкающей стены
        XYZ intersectionPoint = null;
        double minDistance = double.MaxValue;

        foreach (Reference r in sideRefs)
        {
            Face face = adjacentWall.Document.GetElement(r)?.GetGeometryObjectFromReference(r) as Face;
            if (face == null) continue;

            // Находим пересечение линии продления с краями грани
            foreach (EdgeArray ea in face.EdgeLoops)
            {
                foreach (Edge e in ea)
                {
                    Curve c = e.AsCurve();
                    if (c == null || !c.IsBound) continue;

                    // Ищем горизонтальные (в плане) края
                    if (Math.Abs(c.GetEndPoint(0).Z - c.GetEndPoint(1).Z) < 1e-6)
                    {
                        IntersectionResultArray intersectionResults;
                        SetComparisonResult intersection = extensionLine.Intersect(c, out intersectionResults);

                        if (intersection != SetComparisonResult.Disjoint && 
                            intersectionResults != null && intersectionResults.Size > 0)
                        {
                            for (int i = 0; i < intersectionResults.Size; i++)
                            {
                                XYZ point = intersectionResults.get_Item(i).XYZPoint;
                                double dist = sourceEndPoint.DistanceTo(point);
                                
                                // Выбираем ближайшую точку пересечения в разумных пределах
                                if (dist < minDistance && dist < 50.0 && dist > 0.01)
                                {
                                    minDistance = dist;
                                    intersectionPoint = point;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (intersectionPoint == null)
        {
            // Если не нашли пересечение, используем стандартное значение
            // Но для острых углов это может быть недостаточно
            // Попробуем вычислить продление геометрически
            return CalculateExtensionLengthGeometric(sourceWall, adjacentWall, endIndex);
        }

        // Вычисляем расстояние от конца оси исходной стены до точки пересечения
        // Расстояние - это длина вектора от sourceEndPoint до intersectionPoint
        double extension = sourceEndPoint.DistanceTo(intersectionPoint);

        // Проверяем, что продление идет в правильном направлении
        XYZ toIntersection = (intersectionPoint - sourceEndPoint).Normalize();
        if (toIntersection.DotProduct(sourceNormal) < 0)
        {
            // Точка находится в неправильном направлении
            // Попробуем вычислить продление геометрически
            return CalculateExtensionLengthGeometric(sourceWall, adjacentWall, endIndex);
        }

        // Проверяем разумность значения продления
        double maxReasonableExtension = GetWallThickness(sourceWall) + GetWallThickness(adjacentWall);
        if (extension > maxReasonableExtension)
        {
            // Слишком большое продление - используем геометрический расчет
            return CalculateExtensionLengthGeometric(sourceWall, adjacentWall, endIndex);
        }

        return extension;
    }
    catch
    {
        return GetWallThickness(sourceWall) / 2.0;
    }
}

            /// <summary>
/// Вычисляет длину продления геометрически для случаев, когда пересечение не найдено
/// Использует угол между стенами и толщину стен
/// </summary>
private static double CalculateExtensionLengthGeometric(Wall sourceWall, Wall adjacentWall, int endIndex)
{
    if (sourceWall == null || adjacentWall == null)
    {
        return GetWallThickness(sourceWall) / 2.0;
    }

    try
    {
        LocationCurve sourceLocation = sourceWall.Location as LocationCurve;
        LocationCurve adjacentLocation = adjacentWall.Location as LocationCurve;
        
        if (sourceLocation == null || !(sourceLocation.Curve is Line sourceAxis))
            return GetWallThickness(sourceWall) / 2.0;
        
        if (adjacentLocation == null || !(adjacentLocation.Curve is Line adjacentAxis))
            return GetWallThickness(sourceWall) / 2.0;

        XYZ sourceEndPoint = (endIndex == 0) ? sourceAxis.GetEndPoint(0) : sourceAxis.GetEndPoint(1);
        XYZ sourceDir = (sourceAxis.GetEndPoint(1) - sourceAxis.GetEndPoint(0)).Normalize();
        if (endIndex == 0) sourceDir = -sourceDir;

        XYZ adjacentStart = adjacentAxis.GetEndPoint(0);
        XYZ adjacentEnd = adjacentAxis.GetEndPoint(1);
        XYZ adjacentDir = (adjacentEnd - adjacentStart).Normalize();
        
        // Определяем, какой конец примыкающей стены ближе
        double distToStart = sourceEndPoint.DistanceTo(adjacentStart);
        double distToEnd = sourceEndPoint.DistanceTo(adjacentEnd);
        if (distToStart > distToEnd) adjacentDir = -adjacentDir;

        // Вычисляем угол между стенами БЕЗ потери информации о направлении
        // Math.Abs убивает информацию о внутренней/внешней стороне и выпуклости/вогнутости угла
        double dotProduct = sourceDir.DotProduct(adjacentDir);
        double angle = Math.Acos(Math.Min(1.0, Math.Max(-1.0, dotProduct)));
        
        // Определяем направление угла через CrossProduct (для выпуклых/вогнутых углов)
        // CrossProduct.Z покажет направление поворота
        XYZ crossProduct = sourceDir.CrossProduct(adjacentDir);
        // Знак Z компоненты показывает направление поворота в плоскости XY
        
        // Если угол очень мал (стены почти параллельны) или очень большой (почти противоположны)
        if (angle < 0.1 || angle > Math.PI - 0.1)
        {
            return GetWallThickness(sourceWall) / 2.0;
        }

        // Вычисляем продление с учетом угла и толщины стен
        double adjacentHalfThickness = GetWallThickness(adjacentWall) / 2.0;
        
        // Продление оси исходной стены до внешней грани примыкающей стены
        // Внешняя грань - это параллельное смещение оси на adjacentHalfThickness
        // Правильная формула: extension = offset / sin(angle)
        double sinAngle = Math.Sin(angle);
        
        if (sinAngle < 1e-6)
        {
            // Угол слишком мал или близок к 0/180 - стены почти параллельны
            return GetWallThickness(sourceWall) / 2.0;
        }

        // Продление = половина толщины примыкающей стены / sin(угла между осями)
        double extension = adjacentHalfThickness / sinAngle;
        
        // Ограничиваем максимальное продление разумным значением
        // При очень малых углах sin(angle) стремится к 0, что дает огромное продление
        double maxExtension = adjacentHalfThickness * 50.0; // разумный предел для очень острых углов
        return Math.Min(extension, maxExtension);
    }
    catch
    {
        return GetWallThickness(sourceWall) / 2.0;
    }
}

/// <summary>
/// Получает конечную точку внешней грани стены в указанной точке на оси стены
/// </summary>
private static XYZ GetExteriorFaceEndPoint(Wall wall, XYZ axisPoint)
{
    if (wall == null || wall.Document == null || axisPoint == null)
        return null;

    try
    {
        var sideRefs = HostObjectUtils.GetSideFaces(wall, ShellLayerType.Exterior);
        if (sideRefs == null || sideRefs.Count == 0)
            return null;

        LocationCurve wallLocation = wall.Location as LocationCurve;
        if (wallLocation == null || !(wallLocation.Curve is Line wallAxis))
            return null;

        // Определяем направление нормали к стене (наружу)
        XYZ wallDir = (wallAxis.GetEndPoint(1) - wallAxis.GetEndPoint(0)).Normalize();
        XYZ wallNormal = new XYZ(-wallDir.Y, wallDir.X, 0.0).Normalize();
        
        // Определяем, какая нормаль указывает наружу
        // Для этого находим ближайшую точку на оси к axisPoint
        double t = (axisPoint - wallAxis.GetEndPoint(0)).DotProduct(wallDir);
        t = Math.Max(0, Math.Min(wallAxis.Length, t));
        XYZ closestPointOnAxis = wallAxis.GetEndPoint(0) + wallDir * t;
        
        // Получаем внешнюю грань и находим ближайшую точку на её краях
        double minDistance = double.MaxValue;
        XYZ closestPoint = null;
        double halfWidth = GetWallThickness(wall) / 2.0;
        
        foreach (Reference r in sideRefs)
        {
            Face face = wall.Document.GetElement(r)?.GetGeometryObjectFromReference(r) as Face;
            if (face == null) continue;

            // Находим ближайшую точку на краях грани к точке на оси
            foreach (EdgeArray ea in face.EdgeLoops)
            {
                foreach (Edge e in ea)
                {
                    Curve c = e.AsCurve();
                    if (c == null || !c.IsBound) continue;

                    // Ищем горизонтальные (в плане) края
                    if (Math.Abs(c.GetEndPoint(0).Z - c.GetEndPoint(1).Z) < 1e-6)
                    {
                        // Находим ближайшую точку на краю к axisPoint
                        // Используем простой алгоритм: проверяем оба конца и середину
                        XYZ p0 = c.GetEndPoint(0);
                        XYZ p1 = c.GetEndPoint(1);
                        XYZ mid = c.Evaluate(0.5, true);
                        
                        double d0 = axisPoint.DistanceTo(p0);
                        double d1 = axisPoint.DistanceTo(p1);
                        double dMid = axisPoint.DistanceTo(mid);
                        
                        // Выбираем ближайшую точку
                        if (d0 < minDistance && d0 < halfWidth * 3.0)
                        {
                            minDistance = d0;
                            closestPoint = p0;
                        }
                        if (d1 < minDistance && d1 < halfWidth * 3.0)
                        {
                            minDistance = d1;
                            closestPoint = p1;
                        }
                        if (dMid < minDistance && dMid < halfWidth * 3.0)
                        {
                            minDistance = dMid;
                            closestPoint = mid;
                        }
                        
                        // Также проверяем точки вдоль края с шагом
                        for (int i = 0; i <= 10; i++)
                        {
                            double param = i / 10.0;
                            XYZ pointOnEdge = c.Evaluate(param, true);
                            double dist = axisPoint.DistanceTo(pointOnEdge);
                            
                            if (dist < minDistance && dist < halfWidth * 3.0)
                            {
                                minDistance = dist;
                                closestPoint = pointOnEdge;
                            }
                        }
                    }
                }
            }
        }
        
        return closestPoint;
    }
    catch { }

    return null;
}

/// <summary>
/// Получает горизонтальные (плановые) края внешней грани стены
/// </summary>
private static List<Line> GetExteriorFaceEdges(Wall wall)
{
    List<Line> edges = new List<Line>();
    
    if (wall == null || wall.Document == null)
        return edges;

    try
    {
        var sideRefs = HostObjectUtils.GetSideFaces(wall, ShellLayerType.Exterior);
        if (sideRefs == null || sideRefs.Count == 0)
            return edges;

        foreach (Reference r in sideRefs)
        {
            Face face = wall.Document.GetElement(r)?.GetGeometryObjectFromReference(r) as Face;
            if (face == null) continue;

            foreach (EdgeArray ea in face.EdgeLoops)
            {
                foreach (Edge e in ea)
                {
                    Curve c = e.AsCurve();
                    if (c == null || !c.IsBound) continue;

                    // Ищем горизонтальные (в плане) края
                    if (Math.Abs(c.GetEndPoint(0).Z - c.GetEndPoint(1).Z) < 1e-6)
                    {
                        if (c is Line line)
                        {
                            edges.Add(line);
                        }
                    }
                }
            }
        }
    }
    catch { }

    return edges;
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

/// <summary>
/// Обрезает кривую, если она пересекается с существующими стенами
/// Возвращает обрезанную кривую или null, если кривая полностью пересекается
/// </summary>
private static Curve TrimCurveAgainstExistingWalls(Document doc, Curve curve, Wall excludeWall, HashSet<ElementId> excludeCreatedWalls = null)
{
    if (doc == null || curve == null || !(curve is Line line))
        return curve;

    if (excludeWall == null)
        return curve;

    try
    {
        // Получаем все стены в документе, кроме исключаемой и созданных в этой транзакции
        FilteredElementCollector collector = new FilteredElementCollector(doc)
            .OfClass(typeof(Wall))
            .WhereElementIsNotElementType();

        XYZ start = line.GetEndPoint(0);
        XYZ end = line.GetEndPoint(1);
        XYZ dir = (end - start).Normalize();
        
        double minParam = 0.0;
        double maxParam = line.Length;
        const double minLength = 0.1; // Минимальная длина стены (10 см)

        foreach (Wall existingWall in collector.Cast<Wall>())
        {
            // Пропускаем исключаемую стену
            if (existingWall.Id == excludeWall.Id)
                continue;
            
            // Пропускаем созданные в этой транзакции стены (они могут пересекать друг друга)
            if (excludeCreatedWalls != null && excludeCreatedWalls.Contains(existingWall.Id))
                continue;

            LocationCurve existingLocation = existingWall.Location as LocationCurve;
            if (existingLocation == null || existingLocation.Curve == null)
                continue;

            if (!(existingLocation.Curve is Line existingLine))
                continue;

            // Проверяем пересечение осей
            IntersectionResultArray intersectionResults;
            SetComparisonResult intersection = line.Intersect(existingLine, out intersectionResults);

                // Если оси не пересекаются, проверяем, входит ли наша стена в существующую
            if (intersection == SetComparisonResult.Disjoint)
            {
                XYZ existingStart = existingLine.GetEndPoint(0);
                XYZ existingEnd = existingLine.GetEndPoint(1);
                XYZ existingDir = (existingEnd - existingStart).Normalize();
                double existingLength = existingLine.Length;
                
                // Проверяем, перпендикулярны ли стены (строгая проверка через угол)
                double dotProduct = Math.Abs(dir.DotProduct(existingDir));
                double angle = Math.Acos(Math.Min(1.0, Math.Max(-1.0, dotProduct)));
                double angleDegrees = angle * 180.0 / Math.PI;
                
                // Проверяем, что угол близок к 90 градусам (85-95 градусов)
                const double minAngleDegrees = 85.0;
                const double maxAngleDegrees = 95.0;
                bool isPerpendicular = (angleDegrees >= minAngleDegrees && angleDegrees <= maxAngleDegrees);
                
                // Если стены не перпендикулярны, пропускаем
                if (!isPerpendicular)
                {
                    continue;
                }
                
                double existingHalfThickness = GetWallThickness(existingWall) / 2.0;
                
                // Определяем нормаль к существующей стене
                XYZ existingNormal = new XYZ(-existingDir.Y, existingDir.X, 0.0).Normalize();
                
                // Вычисляем расстояние от нашей стены до оси существующей
                double distToExistingAxis = Math.Abs((start - existingStart).DotProduct(existingNormal));
                
                // Если наша стена находится близко к оси существующей (с учетом толщины)
                if (distToExistingAxis < existingHalfThickness + 0.1)
                {
                    // Проецируем концы нашей стены на ось существующей
                    double tStart = (start - existingStart).DotProduct(existingDir);
                    double tEnd = (end - existingStart).DotProduct(existingDir);
                    
                    // Проверяем, входит ли наша стена в существующую
                    // Если хотя бы один конец нашей стены попадает внутрь существующей
                    bool startInside = (tStart > -0.1 && tStart < existingLength + 0.1);
                    bool endInside = (tEnd > -0.1 && tEnd < existingLength + 0.1);
                    
                    if (startInside || endInside)
                    {
                        // Проецируем границы существующей стены на нашу ось
                        double paramExistingStart = (existingStart - start).DotProduct(dir);
                        double paramExistingEnd = (existingEnd - start).DotProduct(dir);
                        
                        // Определяем, с какой стороны существующей стены находится наша стена
                        XYZ ourMid = (start + end) / 2.0;
                        XYZ fromExistingToOur = (ourMid - existingStart);
                        double side = fromExistingToOur.DotProduct(existingNormal);
                        
                        // Вычисляем параметры внешней грани существующей стены на нашей оси
                        // Внешняя грань находится на расстоянии halfThickness от оси
                        double offsetToExterior = (side > 0) ? existingHalfThickness : -existingHalfThickness;
                        double paramExteriorStart = paramExistingStart + offsetToExterior;
                        double paramExteriorEnd = paramExistingEnd + offsetToExterior;
                        
                        // Для перпендикулярных стен обрезаем до внешней грани существующей стены
                        // Учитываем толщину стены - обрезаем на расстоянии halfThickness от оси
                        // Используем уже вычисленные paramExteriorStart и paramExteriorEnd
                        if (startInside)
                        {
                            // Обрезаем начало до ближайшей внешней грани
                            double distToStart = Math.Abs(paramExteriorStart - minParam);
                            double distToEnd = Math.Abs(paramExteriorEnd - minParam);
                            double nearestExterior = (distToStart < distToEnd) ? paramExteriorStart : paramExteriorEnd;
                            
                            if (nearestExterior > minParam && nearestExterior < maxParam)
                            {
                                minParam = nearestExterior;
                            }
                        }
                        
                        if (endInside)
                        {
                            // Обрезаем конец до ближайшей внешней грани
                            double distToStart = Math.Abs(paramExteriorStart - maxParam);
                            double distToEnd = Math.Abs(paramExteriorEnd - maxParam);
                            double nearestExterior = (distToStart < distToEnd) ? paramExteriorStart : paramExteriorEnd;
                            
                            if (nearestExterior > minParam && nearestExterior < maxParam)
                            {
                                maxParam = nearestExterior;
                            }
                        }
                    }
                }
                
                continue;
            }

            // Обрабатываем точки пересечения
            if (intersectionResults != null && intersectionResults.Size > 0)
            {
                // Проверяем, перпендикулярны ли стены (строгая проверка через угол)
                XYZ existingStart = existingLine.GetEndPoint(0);
                XYZ existingEnd = existingLine.GetEndPoint(1);
                XYZ existingDir = (existingEnd - existingStart).Normalize();
                double existingLength = existingLine.Length;
                
                // Вычисляем угол между стенами напрямую
                double dotProduct = Math.Abs(dir.DotProduct(existingDir));
                double angle = Math.Acos(Math.Min(1.0, Math.Max(-1.0, dotProduct)));
                double angleDegrees = angle * 180.0 / Math.PI;
                
                // Проверяем, что угол близок к 90 градусам (85-95 градусов для Г-образных комнат)
                const double minAngleDegrees = 85.0;
                const double maxAngleDegrees = 95.0;
                bool isPerpendicular = (angleDegrees >= minAngleDegrees && angleDegrees <= maxAngleDegrees);
                
                if (isPerpendicular)
                {
                    // Стены перпендикулярны - обрезаем до внешней границы существующей стены
                    double existingHalfThickness = GetWallThickness(existingWall) / 2.0;
                    
                    // Определяем нормаль к существующей стене
                    XYZ existingNormal = new XYZ(-existingDir.Y, existingDir.X, 0.0).Normalize();
                    
                    // Используем точку пересечения осей
                    XYZ intersectionPoint = intersectionResults.get_Item(0).XYZPoint;
                    
                    // Проецируем концы нашей стены на ось существующей
                    double tStart = (start - existingStart).DotProduct(existingDir);
                    double tEnd = (end - existingStart).DotProduct(existingDir);
                    
                    // Проецируем концы нашей стены на ось существующей
                    bool startInside = (tStart >= -0.1 && tStart <= existingLength + 0.1);
                    bool endInside = (tEnd >= -0.1 && tEnd <= existingLength + 0.1);
                    
                    // Проецируем концы существующей стены на нашу ось
                    double paramExistingStart = (existingStart - start).DotProduct(dir);
                    double paramExistingEnd = (existingEnd - start).DotProduct(dir);
                    
                    // Проверяем, находится ли наша стена внутри П-образной формы
                    // Для этого проверяем, находится ли она между концами существующей стены
                    // и параллельна ли она существующей стене (но это перпендикулярный случай)
                    // Для П-образных комнат: если наша стена проходит через всю ширину существующей стены,
                    // обрезаем до концов существующей стены, а не до внешней грани
                    
                    // Определяем, с какой стороны существующей стены находится наша стена
                    XYZ fromIntersectionToOur = (start - intersectionPoint);
                    double side = fromIntersectionToOur.DotProduct(existingNormal);
                    
                    // Вычисляем смещение до внешней границы существующей стены
                    double offsetToExterior = (side > 0) ? existingHalfThickness : -existingHalfThickness;
                    
                    // Вычисляем параметры внешней грани существующей стены на нашей оси
                    double paramExteriorStart = paramExistingStart + offsetToExterior;
                    double paramExteriorEnd = paramExistingEnd + offsetToExterior;
                    
                    // Для перпендикулярных стен определяем тип комнаты и применяем соответствующую логику обрезки
                    if (startInside || endInside)
                    {
                        // Проверяем, является ли это П-образной формой
                        // П-образная форма: стена проходит через существующую стену (один конец слева, другой справа)
                        bool startOutsideLeft = (tStart < -0.1);
                        bool startOutsideRight = (tStart > existingLength + 0.1);
                        bool endOutsideLeft = (tEnd < -0.1);
                        bool endOutsideRight = (tEnd > existingLength + 0.1);
                        
                        bool passesThrough = (startOutsideLeft && endOutsideRight) || (startOutsideRight && endOutsideLeft);
                        bool oneInsideOneOutside = (startInside && (endOutsideLeft || endOutsideRight)) || 
                                                   (endInside && (startOutsideLeft || startOutsideRight));
                        
                        bool isUShape = passesThrough || oneInsideOneOutside;
                        
                        if (isUShape)
                        {
                            // П-образная форма - обрезаем до концов существующей стены
                            if (startInside)
                            {
                                // Начало внутри - обрезаем до ближайшего конца существующей стены
                                double distToStart = Math.Abs(paramExistingStart - minParam);
                                double distToEnd = Math.Abs(paramExistingEnd - minParam);
                                double nearestBoundary = (distToStart < distToEnd) ? paramExistingStart : paramExistingEnd;
                                
                                if (nearestBoundary > minParam && nearestBoundary < maxParam)
                                {
                                    minParam = nearestBoundary;
                                }
                            }
                            
                            if (endInside)
                            {
                                // Конец внутри - обрезаем до ближайшего конца существующей стены
                                double distToStart = Math.Abs(paramExistingStart - maxParam);
                                double distToEnd = Math.Abs(paramExistingEnd - maxParam);
                                double nearestBoundary = (distToStart < distToEnd) ? paramExistingStart : paramExistingEnd;
                                
                                if (nearestBoundary > minParam && nearestBoundary < maxParam)
                                {
                                    maxParam = nearestBoundary;
                                }
                            }
                        }
                        else
                        {
                            // Г-образная форма - обрезаем до внешней грани существующей стены
                            if (startInside && endInside)
                            {
                                // Оба конца входят - обрезаем до внешней грани с той стороны, которая ближе
                                double distToStartMin = Math.Abs(paramExteriorStart - minParam);
                                double distToEndMin = Math.Abs(paramExteriorEnd - minParam);
                                double distToStartMax = Math.Abs(paramExteriorStart - maxParam);
                                double distToEndMax = Math.Abs(paramExteriorEnd - maxParam);
                                
                                double minDistToMin = Math.Min(distToStartMin, distToEndMin);
                                double minDistToMax = Math.Min(distToStartMax, distToEndMax);
                                
                                if (minDistToMin < minDistToMax)
                                {
                                    double nearestExterior = (distToStartMin < distToEndMin) ? paramExteriorStart : paramExteriorEnd;
                                    if (nearestExterior > minParam && nearestExterior < maxParam)
                                    {
                                        minParam = nearestExterior;
                                    }
                                }
                                else
                                {
                                    double nearestExterior = (distToStartMax < distToEndMax) ? paramExteriorStart : paramExteriorEnd;
                                    if (nearestExterior > minParam && nearestExterior < maxParam)
                                    {
                                        maxParam = nearestExterior;
                                    }
                                }
                            }
                            else if (startInside)
                            {
                                // Только начало входит - обрезаем начало до внешней грани
                                double distToStart = Math.Abs(paramExteriorStart - minParam);
                                double distToEnd = Math.Abs(paramExteriorEnd - minParam);
                                double nearestExterior = (distToStart < distToEnd) ? paramExteriorStart : paramExteriorEnd;
                                
                                if (nearestExterior > minParam && nearestExterior < maxParam)
                                {
                                    minParam = nearestExterior;
                                }
                            }
                            else if (endInside)
                            {
                                // Только конец входит - обрезаем конец до внешней грани
                                double distToStart = Math.Abs(paramExteriorStart - maxParam);
                                double distToEnd = Math.Abs(paramExteriorEnd - maxParam);
                                double nearestExterior = (distToStart < distToEnd) ? paramExteriorStart : paramExteriorEnd;
                                
                                if (nearestExterior > minParam && nearestExterior < maxParam)
                                {
                                    maxParam = nearestExterior;
                                }
                            }
                        }
                    }
                }
                else
                {
                    // Стены не перпендикулярны - обрабатываем как обычное пересечение
                    // Для острых/тупых углов просто обрезаем до точки пересечения
                    for (int i = 0; i < intersectionResults.Size; i++)
                    {
                        XYZ intersectionPoint = intersectionResults.get_Item(i).XYZPoint;
                        double param = (intersectionPoint - start).DotProduct(dir);
                        
                        if (param <= minParam)
                        {
                            minParam = param;
                        }
                        else if (param >= maxParam)
                        {
                            maxParam = param;
                        }
                        // Если точка пересечения внутри кривой, обрезаем до неё
                        else
                        {
                            // Определяем, с какой стороны обрезать
                            // Обрезаем ту сторону, которая ближе к точке пересечения
                            double distToStart = param - minParam;
                            double distToEnd = maxParam - param;
                            
                            if (distToStart < distToEnd)
                            {
                                minParam = param;
                            }
                            else
                            {
                                maxParam = param;
                            }
                        }
                    }
                }
            }
            // Если кривые перекрываются полностью
            else if (intersection == SetComparisonResult.Overlap || 
                     intersection == SetComparisonResult.Subset || 
                     intersection == SetComparisonResult.Superset)
            {
                // Проверяем, перекрывается ли наша кривая с существующей
                // Если да, обрезаем до границ существующей стены
                XYZ existingStart = existingLine.GetEndPoint(0);
                XYZ existingEnd = existingLine.GetEndPoint(1);
                
                double paramExistingStart = (existingStart - start).DotProduct(dir);
                double paramExistingEnd = (existingEnd - start).DotProduct(dir);
                
                // Если наша кривая полностью внутри существующей - не создаем
                if (paramExistingStart <= minParam && paramExistingEnd >= maxParam)
                {
                    return null;
                }
                
                // Обрезаем перекрывающиеся части (точно до границ, без отступа)
                if (paramExistingStart > minParam && paramExistingStart < maxParam)
                {
                    maxParam = Math.Min(maxParam, paramExistingStart);
                }
                if (paramExistingEnd > minParam && paramExistingEnd < maxParam)
                {
                    minParam = Math.Max(minParam, paramExistingEnd);
                }
            }
        }

        // Проверяем, осталась ли достаточная длина
        if (maxParam - minParam < minLength)
        {
            return null;
        }

        // Создаем обрезанную кривую
        XYZ newStart = start + dir * minParam;
        XYZ newEnd = start + dir * maxParam;
        
        return Line.CreateBound(newStart, newEnd);
    }
    catch
    {
        return curve;
    }
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
        /// Проверяет, является ли стена внешней по выбранным помещениям:
        /// true — хотя бы с одной стороны нет помещения (ставим внешнюю стену);
        /// false — помещения есть с обеих сторон (перегородка, не создаем).
        /// </summary>
        private static bool IsExternalWall(Document doc, Wall wall, List<BoundarySegmentData> segmentDataList)
        {
            if (wall == null)
                return true; // нет данных — считаем внешней

            try
            {
                if (!(wall.Location is LocationCurve lc) || !(lc.Curve is Line wallLine))
                    return true;

                // Если нет данных о выбранных помещениях — считаем стену внешней
                if (segmentDataList == null || segmentDataList.Count == 0)
                    return true;

                XYZ wallNormal = GetWallFaceNormal(wall);

                bool hasRoomOnPositiveSide = false;
                bool hasRoomOnNegativeSide = false;

                foreach (var segData in segmentDataList)
                {
                    if (segData.Curve == null || segData.Room == null)
                        continue;

                    // Берем середину boundary segment
                    XYZ midPoint = segData.Curve.Evaluate(0.5, true);

                    // Проецируем на ось стены
                    XYZ closestPointOnWall = wallLine.Project(midPoint)?.XYZPoint ?? wallLine.Evaluate(0.5, true);

                    // Вектор от стены к помещению
                    XYZ fromWallToRoom = (midPoint - closestPointOnWall).Normalize();

                    // Определяем сторону по скалярному произведению с нормалью
                    double dotProduct = wallNormal.DotProduct(fromWallToRoom);

                    if (dotProduct > 0.1) // помещение с положительной стороны
                        hasRoomOnPositiveSide = true;
                    else if (dotProduct < -0.1) // помещение с отрицательной стороны
                        hasRoomOnNegativeSide = true;

                    if (hasRoomOnPositiveSide && hasRoomOnNegativeSide)
                        break;
                }

                // Если помещения с обеих сторон — перегородка (false), иначе внешняя (true)
                return !(hasRoomOnPositiveSide && hasRoomOnNegativeSide);
            }
            catch
            {
                // В случае ошибки считаем стену внешней (создаем)
                return true;
            }
        }

        /// <summary>
        /// Создает внешние стены для нескольких помещений.
        /// Если стена используется несколькими помещениями, она разделяется на сегменты по границам помещений.
        /// </summary>
        public static int CreateExternalWallsFromRooms(Document doc, List<Room> rooms, WallType wallType)
        {
            if (doc == null || rooms == null || rooms.Count == 0 || wallType == null)
                return 0;

            int created = 0;
            // Словарь соответствия внутренних и внешних стен для создания проемов
            // Ключ: Tuple<ElementId внутренней стены, ElementId помещения>
            // Значение: Список внешних стен (может быть несколько, если стена разделена на сегменты)
            Dictionary<Tuple<ElementId, ElementId>, List<Wall>> innerWallRoomToExternalWallsMap = 
                new Dictionary<Tuple<ElementId, ElementId>, List<Wall>>();

            try
            {
                Log(doc, $"Начинаем создание внешних стен для {rooms.Count} помещений");

                // Собираем все стены и их boundary segments от всех помещений
                Dictionary<ElementId, List<BoundarySegmentData>> segmentsByWall = 
                    new Dictionary<ElementId, List<BoundarySegmentData>>();

                SpatialElementBoundaryOptions options = new SpatialElementBoundaryOptions();

                foreach (Room room in rooms)
                {
                    IList<IList<BoundarySegment>> boundaryLoops = room.GetBoundarySegments(options);
                    if (boundaryLoops == null) continue;

                    foreach (IList<BoundarySegment> loop in boundaryLoops)
                    {
                        foreach (BoundarySegment segment in loop)
                        {
                            Element boundaryElement = doc.GetElement(segment.ElementId);
                            if (boundaryElement is Wall wall)
                            {
                                if (!segmentsByWall.ContainsKey(wall.Id))
                                {
                                    segmentsByWall[wall.Id] = new List<BoundarySegmentData>();
                                }

                                segmentsByWall[wall.Id].Add(new BoundarySegmentData
                                {
                                    Segment = segment,
                                    Room = room,
                                    Curve = segment.GetCurve()
                                });
                            }
                        }
                    }
                }

                Log(doc, $"Найдено уникальных стен: {segmentsByWall.Count}");

                // Список ID созданных стен для исключения из проверки пересечений
                HashSet<ElementId> createdWallIds = new HashSet<ElementId>();

                // Обрабатываем каждую стену
                foreach (var kvp in segmentsByWall)
                {
                    Wall innerWall = doc.GetElement(kvp.Key) as Wall;
                    if (innerWall == null) continue;

                    List<BoundarySegmentData> segmentDataList = kvp.Value;

                    // Получаем осевую линию стены
                    LocationCurve wallLocation = innerWall.Location as LocationCurve;
                    if (wallLocation == null || wallLocation.Curve == null || !(wallLocation.Curve is Line))
                    {
                        Log(doc, $"Стена {innerWall.Id} не имеет подходящей осевой линии");
                        continue;
                    }
                    Line axisLine = wallLocation.Curve as Line;

                    // Проверяем, является ли стена внешней (хотя бы с одной стороны нет помещения)
                    if (!IsExternalWall(doc, innerWall, segmentDataList))
                    {
                        Log(doc, $"Стена {innerWall.Id} имеет помещения с обеих сторон, пропуск (перегородка)");
                        continue;
                    }

                    List<WallSegment> wallSegments = DivideWallByRoomBoundaries(doc, innerWall, axisLine, segmentDataList, rooms);
                    if (wallSegments == null || wallSegments.Count == 0)
                        continue;

                    // Получаем параметры стены
                    Level level = GetWallLevel(innerWall);
                    double height = GetWallHeight(innerWall);
                    if (level == null) continue;

                    double innerThickness = GetWallThickness(innerWall);
                    double externalThickness = GetWallTypeThickness(wallType);
                    double offsetDistance = (innerThickness / 2.0) + (externalThickness / 2.0);

                    // Получаем параметры оси исходной стены для определения концов
                    XYZ axisStart = axisLine.GetEndPoint(0);
                    XYZ axisDir = (axisLine.GetEndPoint(1) - axisStart).Normalize();
                    double axisLength = axisLine.Length;

                    // Собираем точки разделения из сегментов
                    HashSet<double> splitPoints = new HashSet<double>();
                    foreach (var seg in wallSegments)
                    {
                        XYZ segStart = seg.Curve.GetEndPoint(0);
                        XYZ segEnd = seg.Curve.GetEndPoint(1);
                        
                        double tStart = (segStart - axisStart).DotProduct(axisDir);
                        double tEnd = (segEnd - axisStart).DotProduct(axisDir);
                        
                        // Добавляем внутренние точки (не концы стены)
                        const double tolerance = 0.01;
                        if (tStart > tolerance && tStart < axisLength - tolerance)
                            splitPoints.Add(tStart);
                        if (tEnd > tolerance && tEnd < axisLength - tolerance)
                            splitPoints.Add(tEnd);
                    }

                    // Создаем внешнюю стену для каждого сегмента
                    for (int segIndex = 0; segIndex < wallSegments.Count; segIndex++)
                    {
                        WallSegment segment = wallSegments[segIndex];
                        
                        XYZ segStart = segment.Curve.GetEndPoint(0);
                        XYZ segEnd = segment.Curve.GetEndPoint(1);
                        
                        double tStart = (segStart - axisStart).DotProduct(axisDir);
                        double tEnd = (segEnd - axisStart).DotProduct(axisDir);
                        
                        const double tolerance = 0.01;
                        bool startIsSplitPoint = splitPoints.Contains(tStart) || Math.Abs(tStart) < tolerance;
                        bool endIsSplitPoint = splitPoints.Contains(tEnd) || Math.Abs(tEnd - axisLength) < tolerance;
                        bool startIsWallEnd = Math.Abs(tStart) < tolerance;
                        bool endIsWallEnd = Math.Abs(tEnd - axisLength) < tolerance;
                        
                        // Определяем направление наружу для этого сегмента
                        XYZ outwardNormal = GetOutwardNormalFromRoom(innerWall, segment.Curve, segment.Room);
                        XYZ externalStart = segStart + outwardNormal * offsetDistance;
                        XYZ externalEnd = segEnd + outwardNormal * offsetDistance;
                        
                        Curve externalCurve = Line.CreateBound(externalStart, externalEnd);
                        
                        // Проверяем, не пересекает ли создаваемая стена комнату
                        // Если пересекает, пробуем противоположное направление
                        // Передаем boundary curve сегмента для более точной проверки
                        if (DoesWallIntersectRoom(externalCurve, innerWall, segment.Room, segment.Curve))
                        {
                            // Пробуем противоположное направление
                            XYZ oppositeNormal = -outwardNormal;
                            XYZ oppositeStart = segStart + oppositeNormal * offsetDistance;
                            XYZ oppositeEnd = segEnd + oppositeNormal * offsetDistance;
                            Curve oppositeCurve = Line.CreateBound(oppositeStart, oppositeEnd);
                            
                            // Если противоположное направление не пересекает комнату, используем его
                            if (!DoesWallIntersectRoom(oppositeCurve, innerWall, segment.Room, segment.Curve))
                            {
                                outwardNormal = oppositeNormal;
                                externalStart = oppositeStart;
                                externalEnd = oppositeEnd;
                                externalCurve = oppositeCurve;
                            }
                            // Иначе оставляем исходное направление (может быть случай, когда оба пересекают)
                            else
                            {
                                outwardNormal = -outwardNormal;
                                externalStart = segStart + outwardNormal * offsetDistance;
                                externalEnd = segEnd + outwardNormal * offsetDistance;
                                externalCurve = Line.CreateBound(externalStart, externalEnd);
                            }
                        }
                        
                        // Растягиваем только если край является концом исходной стены, а не точкой разделения
                        if (startIsWallEnd || endIsWallEnd)
                        {
                            externalCurve = ExtendToWallEnds(innerWall, externalCurve);
                            externalCurve = ExtendCurveToJoinedWalls(innerWall, externalCurve);
                            
                            // Если начало - точка разделения, обрезаем его обратно
                            if (!startIsWallEnd && startIsSplitPoint)
                            {
                                // Обрезаем начало до исходной точки
                                XYZ newStart = externalStart;
                                externalCurve = Line.CreateBound(newStart, externalCurve.GetEndPoint(1));
                            }
                            
                            // Если конец - точка разделения, обрезаем его обратно
                            if (!endIsWallEnd && endIsSplitPoint)
                            {
                                // Обрезаем конец до исходной точки
                                XYZ newEnd = externalEnd;
                                externalCurve = Line.CreateBound(externalCurve.GetEndPoint(0), newEnd);
                            }
                        }

                        if (externalCurve == null || externalCurve.Length < 0.01)
                            continue;

                        // Проверяем пересечение с существующими стенами и обрезаем при необходимости
                        // Исключаем из проверки уже созданные в этой транзакции стены
                        externalCurve = TrimCurveAgainstExistingWalls(doc, externalCurve, innerWall, createdWallIds);
                        if (externalCurve == null || externalCurve.Length < 0.01)
                            continue;

                        // Создаем внешнюю стену для сегмента
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
                            DisableWallJoins(externalWall);
                            CopyWallProperties(innerWall, externalWall);
                            
                            // Устанавливаем параметр ADSK_Зона для внешней стены
                            SetZoneParameter(doc, externalWall, null);
                            
                            created++;
                            // Добавляем ID созданной стены в список для исключения из проверки
                            createdWallIds.Add(externalWall.Id);
                            // Сохраняем соответствие между внутренней и внешней стеной с привязкой к помещению
                            // Это важно, когда одна внутренняя стена используется несколькими помещениями
                            // Может быть несколько внешних стен для одной пары (стена, помещение), если стена разделена на сегменты
                            Tuple<ElementId, ElementId> key = new Tuple<ElementId, ElementId>(innerWall.Id, segment.Room.Id);
                            if (!innerWallRoomToExternalWallsMap.ContainsKey(key))
                            {
                                innerWallRoomToExternalWallsMap[key] = new List<Wall>();
                            }
                            innerWallRoomToExternalWallsMap[key].Add(externalWall);
                            Log(doc, $"Создана внешняя стена {externalWall.Id} для сегмента стены {innerWall.Id} помещения {segment.Room.Id} (всего внешних стен для этой пары: {innerWallRoomToExternalWallsMap[key].Count})");
                        }
                    }
                }

                // Соединяем геометрию между внутренними и внешними стенами для автоматического создания проемов
                // Проверяем настройки: создавать ли проемы
                Settings settings = Settings.Load();
                if (settings.CreateOpenings)
                {
                    int totalJoinsCreated = 0;
                    foreach (Room room in rooms)
                    {
                        int joinsCreated = JoinGeometryBetweenWalls(doc, room, innerWallRoomToExternalWallsMap);
                        totalJoinsCreated += joinsCreated;
                    }
                        Log(doc, $"Соединена геометрия для {totalJoinsCreated} пар стен. Проемы будут созданы автоматически.");
                }
                else
                {
                    Log(doc, "Создание проемов отключено в настройках. Пропускаем соединение геометрии.");
                }

                // Устанавливаем окна и двери во внешние стены, если это включено в настройках
                if (settings.CopyOpeningsToExternalWalls)
                {
                    int copiedCount = CopyOpeningsToExternalWalls(doc, rooms, innerWallRoomToExternalWallsMap);
                    if (copiedCount > 0)
                    {
                        Log(doc, $"Установлено {copiedCount} окон/дверей во внешние стены.");
                    }
                }

                Log(doc, $"Всего создано внешних стен: {created}");
            }
            catch (Exception ex)
            {
                Log(doc, $"Ошибка при создании внешних стен: {ex.Message}");
                throw new Exception($"Error creating external walls from rooms: {ex.Message}", ex);
            }

            return created;
        }

            /// <summary>
        /// Данные о boundary segment с привязкой к помещению
        /// </summary>
        private class BoundarySegmentData
        {
            public BoundarySegment Segment { get; set; }
            public Room Room { get; set; }
            public Curve Curve { get; set; }
        }

        /// <summary>
        /// Сегмент стены с привязкой к помещению
        /// </summary>
        private class WallSegment
        {
            public Curve Curve { get; set; }
            public Room Room { get; set; }
        }

        /// <summary>
        /// Проверяет, граничит ли стена с двумя и более помещениями (с разных сторон).
        /// Если да – внешнюю стену не создаём.
        /// </summary>
        private static bool HasRoomsOnBothSides(Document doc, Wall wall)
        {
            try
            {
                if (doc == null || wall == null)
                    return false;

                SpatialElementBoundaryOptions opt = new SpatialElementBoundaryOptions();
                LocationCurve lc = wall.Location as LocationCurve;
                if (lc == null || !(lc.Curve is Line axisLine))
                    return false;

                XYZ axisStart = axisLine.GetEndPoint(0);
                XYZ axisEnd = axisLine.GetEndPoint(1);
                XYZ axisDir = (axisEnd - axisStart).Normalize();
                XYZ wallNormal = GetWallFaceNormal(wall);

                bool hasPos = false;
                bool hasNeg = false;

                var rooms = new FilteredElementCollector(doc)
                    .OfClass(typeof(SpatialElement))
                    .OfType<Room>();

                foreach (var room in rooms)
                {
                    var loops = room.GetBoundarySegments(opt);
                    if (loops == null) continue;
                    foreach (var loop in loops)
                    {
                        foreach (var seg in loop)
                        {
                            if (seg.ElementId != wall.Id) continue;

                            Curve c = seg.GetCurve();
                            if (c == null) continue;
                            // берём середину граничного сегмента комнаты
                            XYZ mid = c.Evaluate(0.5, true);
                            // проекция на ось для плоскости
                            XYZ proj = axisLine.Project(mid)?.XYZPoint ?? axisLine.Evaluate(0.5, true);
                            double side = (mid - proj).DotProduct(wallNormal);

                            if (side > 1e-6) hasPos = true;
                            else if (side < -1e-6) hasNeg = true;

                            if (hasPos && hasNeg)
                                return true;
                        }
                    }
                }

                return false;
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// Разделяет стену на сегменты по точкам пересечения с внутренними стенами (перегородками),
        /// которые граничат с разными выбранными комнатами.
        /// </summary>
        private static List<WallSegment> DivideWallByRoomBoundaries(
            Document doc,
    Wall wall,
            Line axisLine, 
            List<BoundarySegmentData> segmentDataList,
            List<Room> selectedRooms)
        {
            var segments = new List<WallSegment>();

            if (wall == null || axisLine == null || segmentDataList == null || segmentDataList.Count == 0)
            {
                // Если нет данных для разделения, возвращаем всю стену
                if (segmentDataList != null && segmentDataList.Count > 0)
                {
                    segments.Add(new WallSegment
                    {
                        Curve = axisLine,
                        Room = segmentDataList[0].Room
                    });
                }
                return segments;
            }

            try
            {
                XYZ axisStart = axisLine.GetEndPoint(0);
                XYZ axisEnd = axisLine.GetEndPoint(1);
                XYZ axisDir = (axisEnd - axisStart).Normalize();
                double axisLength = axisLine.Length;

                // Группируем boundary segments по комнатам
                Dictionary<ElementId, List<Curve>> segmentsByRoom = new Dictionary<ElementId, List<Curve>>();
                foreach (var segData in segmentDataList)
                {
                    if (segData.Curve == null || segData.Room == null)
                        continue;

                    ElementId roomId = segData.Room.Id;
                    if (!segmentsByRoom.ContainsKey(roomId))
                        segmentsByRoom[roomId] = new List<Curve>();
                    
                    segmentsByRoom[roomId].Add(segData.Curve);
                }

                // Если только одна комната - возвращаем всю стену
                if (segmentsByRoom.Count <= 1)
                {
                    Room singleRoom = segmentDataList[0].Room;
                    segments.Add(new WallSegment
                    {
                        Curve = axisLine,
                        Room = singleRoom
                    });
                    return segments;
                }

                // Находим точки пересечения с внутренними стенами (перегородками),
                // которые граничат с разными выбранными комнатами
                SortedSet<double> intersectionPoints = new SortedSet<double>();
                SpatialElementBoundaryOptions options = new SpatialElementBoundaryOptions();

                // Собираем ID всех выбранных комнат
                HashSet<ElementId> selectedRoomIds = new HashSet<ElementId>(
                    selectedRooms.Select(r => r.Id));

                // Находим все стены в документе, которые пересекают текущую стену
                FilteredElementCollector wallCollector = new FilteredElementCollector(doc)
                    .OfClass(typeof(Wall))
                    .WhereElementIsNotElementType();

                foreach (Wall otherWall in wallCollector.Cast<Wall>())
                {
                    if (otherWall.Id == wall.Id)
                        continue; // Пропускаем саму стену

                    LocationCurve otherLocation = otherWall.Location as LocationCurve;
                    if (otherLocation == null || otherLocation.Curve == null)
            continue;

                    if (!(otherLocation.Curve is Line otherLine))
                        continue;

                    // Проверяем пересечение осей стен
                    SetComparisonResult intersectResult = axisLine.Intersect(otherLine, out IntersectionResultArray intersections);
                    
                    if (intersectResult == SetComparisonResult.Overlap && intersections != null && intersections.Size > 0)
                    {
                        // Стены пересекаются - проверяем, граничит ли эта стена с разными выбранными комнатами
                        HashSet<ElementId> roomsUsingOtherWall = new HashSet<ElementId>();

                        foreach (Room room in selectedRooms)
                        {
                            IList<IList<BoundarySegment>> boundaryLoops = room.GetBoundarySegments(options);
                            if (boundaryLoops == null) continue;

                            foreach (IList<BoundarySegment> loop in boundaryLoops)
                            {
                                foreach (BoundarySegment segment in loop)
                                {
                                    if (segment.ElementId == otherWall.Id)
                                    {
                                        roomsUsingOtherWall.Add(room.Id);
                                        break;
                                    }
                                }
                            }
                        }

                        // Если перегородка граничит с разными выбранными комнатами - это точка разделения
                        if (roomsUsingOtherWall.Count >= 2)
                        {
                            // Берем первую точку пересечения осей
                            IntersectionResult intersection = intersections.get_Item(0);
                            XYZ axisIntersectionPoint = intersection.XYZPoint;

                            // Учитываем толщину перегородки для определения точек пересечения внешних граней
                            double partitionThickness = GetWallThickness(otherWall);
                            double partitionHalfThickness = partitionThickness / 2.0;

                            // Направление перегородки
                            XYZ otherDir = (otherLine.GetEndPoint(1) - otherLine.GetEndPoint(0)).Normalize();
                            // Нормаль к перегородке (перпендикулярно в плоскости XY)
                            XYZ otherNormal = new XYZ(-otherDir.Y, otherDir.X, 0.0).Normalize();

                            // Точки, где внешние грани перегородки пересекают ось внешней стены
                            // Смещаем точку пересечения осей на половину толщины перегородки в обе стороны
                            XYZ outerFace1 = axisIntersectionPoint + otherNormal * partitionHalfThickness;
                            XYZ outerFace2 = axisIntersectionPoint - otherNormal * partitionHalfThickness;

                            // Проецируем эти точки на ось внешней стены
                            double t1 = (outerFace1 - axisStart).DotProduct(axisDir);
                            double t2 = (outerFace2 - axisStart).DotProduct(axisDir);

                            // Берем середину между этими точками как точку разделения
                            double t = (t1 + t2) / 2.0;

                            // Добавляем только внутренние точки (не концы стены)
                            const double edgeTolerance = 0.01;
                            if (t > edgeTolerance && t < axisLength - edgeTolerance)
                            {
                                intersectionPoints.Add(t);
                                Log(wall.Document, $"Найдена точка разделения на стене {wall.Id} от перегородки {otherWall.Id} при t={t:F3} (середина между гранями перегородки)");
                            }
                        }
                    }
                }

                // Если нет точек пересечения - возвращаем всю стену с первой комнатой
                if (intersectionPoints.Count == 0)
                {
                    segments.Add(new WallSegment
                    {
                        Curve = axisLine,
                        Room = segmentDataList[0].Room
                    });
                    return segments;
                }

                // Строим список точек разреза с концами стены
                List<double> splitPoints = new List<double> { 0.0 };
                splitPoints.AddRange(intersectionPoints);
                splitPoints.Add(axisLength);

                // Создаем сегменты между точками пересечения
                for (int i = 0; i < splitPoints.Count - 1; i++)
                {
                    double tStart = splitPoints[i];
                    double tEnd = splitPoints[i + 1];

                    if (tEnd - tStart < 0.01)
                continue;

                    // Находим комнату для этого интервала
                    Room roomForSegment = FindRoomForInterval(
                        axisLine, tStart, tEnd, segmentDataList);

                    if (roomForSegment != null)
                    {
                        XYZ segStart = axisStart + axisDir * tStart;
                        XYZ segEnd = axisStart + axisDir * tEnd;
                        Curve segmentCurve = Line.CreateBound(segStart, segEnd);

                        segments.Add(new WallSegment
                        {
                            Curve = segmentCurve,
                            Room = roomForSegment
                        });
                    }
                }
            }
            catch (Exception ex)
            {
                Log(wall.Document, $"Ошибка при разделении стены: {ex.Message}");
                // В случае ошибки возвращаем всю стену
                if (segmentDataList.Count > 0)
                {
                    segments.Add(new WallSegment
                    {
                        Curve = axisLine,
                        Room = segmentDataList[0].Room
                    });
                }
            }

            return segments;
        }

        /// <summary>
        /// Находит помещение для заданного интервала на оси стены
        /// </summary>
        private static Room FindRoomForInterval(
            Line axisLine, 
            double tStart, 
            double tEnd, 
            List<BoundarySegmentData> segmentDataList)
        {
            XYZ axisStart = axisLine.GetEndPoint(0);
            XYZ axisDir = (axisLine.GetEndPoint(1) - axisStart).Normalize();
            double midT = (tStart + tEnd) / 2.0;
            XYZ midPoint = axisStart + axisDir * midT;

            // Ищем boundary segment, который содержит эту точку
            foreach (var segData in segmentDataList)
            {
                if (segData.Curve == null) continue;

                // Проверяем, находится ли точка вблизи этого boundary segment
                double dist = segData.Curve.Distance(midPoint);
                if (dist < 0.1) // допуск ~30 мм
                {
                    return segData.Room;
                }
            }

            // Если не нашли, возвращаем первое помещение
            return segmentDataList.Count > 0 ? segmentDataList[0].Room : null;
        }

        /// <summary>
        /// Получает все окна и двери из помещения (находящиеся в стенах помещения)
        /// </summary>
        private static List<FamilyInstance> GetWindowsAndDoorsFromRoom(Document doc, Room room)
        {
            List<FamilyInstance> openings = new List<FamilyInstance>();

            try
            {
                // Получаем стены помещения
                SpatialElementBoundaryOptions options = new SpatialElementBoundaryOptions();
                IList<IList<BoundarySegment>> boundaryLoops = room.GetBoundarySegments(options);
                
                if (boundaryLoops == null) return openings;

                HashSet<ElementId> roomWallIds = new HashSet<ElementId>();

                foreach (IList<BoundarySegment> loop in boundaryLoops)
                {
                    foreach (BoundarySegment segment in loop)
                    {
                        Element boundaryElement = doc.GetElement(segment.ElementId);
                        if (boundaryElement is Wall wall)
                        {
                            roomWallIds.Add(wall.Id);
                        }
                    }
                }

                // Получаем все окна и двери в документе
                FilteredElementCollector collector = new FilteredElementCollector(doc)
                    .OfClass(typeof(FamilyInstance))
                    .OfCategory(BuiltInCategory.OST_Windows)
                    .UnionWith(new FilteredElementCollector(doc)
                        .OfClass(typeof(FamilyInstance))
                        .OfCategory(BuiltInCategory.OST_Doors));

                foreach (FamilyInstance instance in collector)
                {
                    // Проверяем, находится ли окно/дверь в стене помещения
                    Wall hostWall = instance.Host as Wall;
                    if (hostWall != null && roomWallIds.Contains(hostWall.Id))
                    {
                        openings.Add(instance);
                        Log(doc, $"Найдено окно/дверь {instance.Id} в стене {hostWall.Id} помещения {room.Id}");
                    }
                }
            }
            catch (Exception ex)
            {
                Log(doc, $"Ошибка при получении окон и дверей из помещения: {ex.Message}");
            }

            return openings;
        }

        /// <summary>
        /// Проверяет, принадлежит ли окно/дверь данному помещению
        /// Использует Room.GetRoom для определения помещения по точке окна/двери
        /// </summary>
        private static bool DoesOpeningBelongToRoom(Document doc, FamilyInstance opening, Room room)
        {
            try
            {
                // Получаем позицию окна/двери
                LocationPoint locationPoint = opening.Location as LocationPoint;
                LocationCurve locationCurve = opening.Location as LocationCurve;
                
                XYZ openingPoint = null;
                if (locationPoint != null)
                {
                    openingPoint = locationPoint.Point;
                }
                else if (locationCurve != null && locationCurve.Curve != null)
                {
                    openingPoint = locationCurve.Curve.Evaluate(0.5, true);
                }

                if (openingPoint == null) return false;

                // Используем Room.GetRoom для определения помещения по точке
                // Смещаем точку немного внутрь от стены, чтобы она точно попала в помещение
                Wall hostWall = opening.Host as Wall;
                if (hostWall != null)
                {
                    LocationCurve wallLocation = hostWall.Location as LocationCurve;
                    if (wallLocation != null && wallLocation.Curve != null)
                    {
                        Curve wallCurve = wallLocation.Curve;
                        // Находим ближайшую точку на стене
                        try
                        {
                            IntersectionResult proj = wallCurve.Project(openingPoint);
                            double param = proj.Parameter;
                            // ВАЖНО: false - param это натуральный параметр (не нормализованный 0..1)
                            XYZ pointOnWall = wallCurve.Evaluate(param, false);
                            
                            // Вычисляем направление от стены к окну/двери
                            XYZ direction = (openingPoint - pointOnWall).Normalize();
                            
                            // Смещаем точку немного внутрь от стены (в направлении окна/двери)
                            // Используем небольшое смещение (около 0.1 фута)
                            XYZ testPoint = openingPoint + direction * 0.1;
                            
                            // Проверяем, в каком помещении находится эта точка
                            Room roomAtPoint = doc.GetRoomAtPoint(testPoint);
                            if (roomAtPoint != null && roomAtPoint.Id == room.Id)
                            {
                                return true;
                            }
                        }
                        catch
                        {
                            // Если не удалось определить через GetRoom, используем fallback
                        }
                    }
                }

                // Fallback: проверяем, находится ли окно/дверь в стене, которая граничит с этим помещением
                // Это менее точный метод, но работает как запасной вариант
                SpatialElementBoundaryOptions options = new SpatialElementBoundaryOptions();
                IList<IList<BoundarySegment>> boundaryLoops = room.GetBoundarySegments(options);
                
                if (boundaryLoops != null)
                {
                    foreach (IList<BoundarySegment> loop in boundaryLoops)
                    {
                        foreach (BoundarySegment segment in loop)
                        {
                            Element boundaryElement = doc.GetElement(segment.ElementId);
                            if (boundaryElement is Wall wall && wall.Id == hostWall.Id)
                            {
                                // Стена граничит с помещением, но нужно проверить, с какой стороны находится окно
                                // Для простоты, если стена граничит с помещением, считаем что окно может принадлежать
                                // Но лучше использовать более точную проверку через GetRoom
                                return true;
                            }
                        }
                    }
                }

                return false;
            }
            catch (Exception ex)
            {
                // В случае ошибки считаем что принадлежит (безопасный вариант)
                Log(doc, $"Ошибка при проверке принадлежности окна/двери {opening.Id} помещению {room.Id}: {ex.Message}");
                return true;
            }
        }

        /// <summary>
        /// Создает проемы во внешних стенах на основе окон и дверей из внутренних стен
        /// Использует подход Room Finisher: соединяет геометрию стен и позволяет проемам автоматически прорезать обе стены
        /// </summary>
        private static int JoinGeometryBetweenWalls(Document doc, Room room, Dictionary<Tuple<ElementId, ElementId>, List<Wall>> innerWallRoomToExternalWallsMap)
        {
            int joinsCreated = 0;
            int openingsProcessed = 0;

            try
            {
                // Получаем окна и двери из помещения
                List<FamilyInstance> openings = GetWindowsAndDoorsFromRoom(doc, room);

                if (openings.Count == 0)
                {
                    Log(doc, $"В помещении {room.Id} не найдено окон и дверей");
                    return 0;
                }

                Log(doc, $"Найдено окон и дверей в помещении {room.Id}: {openings.Count}");

                // Важно: Regenerate перед соединением геометрии, чтобы убедиться, что стены полностью созданы
                doc.Regenerate();

                // Проверяем высоту окон/дверей, чтобы убедиться, что внешние стены достаточно высокие
                double maxOpeningHeight = 0.0;
                foreach (FamilyInstance opening in openings)
                {
                    BoundingBoxXYZ bbox = opening.get_BoundingBox(null);
                    if (bbox != null)
                    {
                        double openingHeight = bbox.Max.Z - bbox.Min.Z;
                        if (openingHeight > maxOpeningHeight)
                            maxOpeningHeight = openingHeight;
                    }
                }
                
                if (maxOpeningHeight > 0)
                {
                    Log(doc, $"Максимальная высота окна/двери: {maxOpeningHeight:F3}");
                }

                // Сначала соединяем геометрию между внутренними и внешними стенами
                // Обрабатываем только стены для текущего помещения
                foreach (var kvp in innerWallRoomToExternalWallsMap)
                {
                    ElementId innerWallId = kvp.Key.Item1;
                    ElementId roomId = kvp.Key.Item2;
                    List<Wall> externalWalls = kvp.Value; // Теперь это список стен

                    // Пропускаем стены, которые не относятся к текущему помещению
                    if (roomId != room.Id)
                    {
                        continue;
                    }

                    Wall innerWall = doc.GetElement(innerWallId) as Wall;
                    if (innerWall == null)
                    {
                        Log(doc, $"Внутренняя стена {innerWallId} не найдена");
                        continue;
                    }
                    
                    Log(doc, $"Обрабатываем внутреннюю стену {innerWall.Id} для помещения {room.Id}: найдено {externalWalls.Count} внешних стен(ы)");
                    
                    // Соединяем геометрию со ВСЕМИ внешними стенами для этой пары (стена, помещение)
                    foreach (Wall externalWall in externalWalls)
                    {
                        try
                        {
                        // Проверяем высоту внешней стены
                        double externalWallHeight = GetWallHeight(externalWall);
                        double innerWallHeight = GetWallHeight(innerWall);
                        Level externalLevel = GetWallLevel(externalWall);
                        Level innerLevel = GetWallLevel(innerWall);
                        
                        double externalTop = externalLevel != null ? externalLevel.Elevation + externalWallHeight : 0;
                        double innerTop = innerLevel != null ? innerLevel.Elevation + innerWallHeight : 0;
                        
                        Log(doc, $"Внутренняя стена {innerWall.Id}: высота {innerWallHeight:F3}, верх {innerTop:F3}");
                        Log(doc, $"Внешняя стена {externalWall.Id}: высота {externalWallHeight:F3}, верх {externalTop:F3}");
                        
                        // ВАЖНО: Проверяем, соединены ли стены
                        // Если стены уже соединены, это может быть соединение от другого помещения
                        // Но нам все равно нужно обработать проемы из ТЕКУЩЕГО помещения
                        bool alreadyJoined = JoinGeometryUtils.AreElementsJoined(doc, innerWall, externalWall);
                        if (alreadyJoined)
                        {
                            Log(doc, $"Стены {innerWall.Id} и {externalWall.Id} уже соединены (возможно, от другого помещения {room.Id})");
                            joinsCreated++; // Считаем, что соединение есть
                        }

                        // Детальная диагностика: проверяем расстояние между стенами
                        LocationCurve innerLocation = innerWall.Location as LocationCurve;
                        LocationCurve externalLocation = externalWall.Location as LocationCurve;
                        
                        double minDist = double.MaxValue;
                        double maxDist = 0;
                        if (innerLocation != null && externalLocation != null)
                        {
                            Curve innerCurve = innerLocation.Curve;
                            Curve externalCurve = externalLocation.Curve;
                            
                            if (innerCurve != null && externalCurve != null)
                            {
                                // Проверяем расстояние в нескольких точках
                                for (int i = 0; i <= 10; i++)
                                {
                                    double param = i / 10.0;
                                    XYZ innerPoint = innerCurve.Evaluate(param, true);
                                    double dist = externalCurve.Distance(innerPoint);
                                    if (dist < minDist) minDist = dist;
                                    if (dist > maxDist) maxDist = dist;
                                }
                                
                                Log(doc, $"Расстояние между стенами {innerWall.Id} и {externalWall.Id}: мин={minDist:F3}, макс={maxDist:F3}");
                            }
                        }

                        // Проверяем, пересекаются ли стены в 3D пространстве
                        BoundingBoxXYZ innerBbox = innerWall.get_BoundingBox(null);
                        BoundingBoxXYZ externalBbox = externalWall.get_BoundingBox(null);
                        
                        bool wallsIntersect = false;
                        if (innerBbox != null && externalBbox != null)
                        {
                            // Проверяем пересечение по X, Y, Z
                            bool xOverlap = innerBbox.Max.X >= externalBbox.Min.X && innerBbox.Min.X <= externalBbox.Max.X;
                            bool yOverlap = innerBbox.Max.Y >= externalBbox.Min.Y && innerBbox.Min.Y <= externalBbox.Max.Y;
                            bool zOverlap = innerBbox.Max.Z >= externalBbox.Min.Z && innerBbox.Min.Z <= externalBbox.Max.Z;
                            
                            wallsIntersect = xOverlap && yOverlap && zOverlap;
                            
                            if (!wallsIntersect)
                            {
                                Log(doc, $"⚠ Стены {innerWall.Id} и {externalWall.Id} НЕ пересекаются в 3D! " +
                                    $"Join Geometry может не сработать.");
                            }
                            else
                            {
                                Log(doc, $"✓ Стены {innerWall.Id} и {externalWall.Id} пересекаются в 3D");
                            }
                        }

                        // Соединяем геометрию между внутренней и внешней стеной
                        // Порядок важен: сначала стена с окнами/дверями (innerWall), затем стена, которая будет резаться (externalWall)
                        if (!alreadyJoined)
                        {
                            try
                            {
                                JoinGeometryUtils.JoinGeometry(doc, innerWall, externalWall);
                                Log(doc, $"Вызван JoinGeometry для стен {innerWall.Id} и {externalWall.Id} помещения {room.Id}");
                            }
                            catch (Exception joinEx)
                            {
                                Log(doc, $"✗ Исключение при JoinGeometry для стен {innerWall.Id} и {externalWall.Id}: {joinEx.Message}");
                                continue;
                            }
                            
                            // Regenerate сразу после соединения для этой пары стен
                            doc.Regenerate();
                            
                            // Проверяем, что соединение прошло успешно
                            if (JoinGeometryUtils.AreElementsJoined(doc, innerWall, externalWall))
                            {
                                joinsCreated++; // Увеличиваем счетчик только когда соединение действительно создано
                                Log(doc, $"✓ Успешно соединена геометрия между внутренней стеной {innerWall.Id} и внешней стеной {externalWall.Id} для помещения {room.Id}");
                            }
                            else
                            {
                                Log(doc, $"✗ JoinGeometry не создал соединение для стен {innerWall.Id} и {externalWall.Id}");
                                continue;
                            }
                        }
                        
                        // Regenerate перед проверкой проемов (важно для обновления геометрии)
                        doc.Regenerate();
                        
                        // Проверяем окна/двери в этой стене и их пересечение с внешней стеной
                        // Это выполняется для всех соединенных стен (независимо от того, были ли они соединены до этого)
                        int openingsInThisWall = 0;
                        foreach (FamilyInstance opening in openings)
                        {
                            Wall hostWall = opening.Host as Wall;
                            if (hostWall != null && hostWall.Id == innerWall.Id)
                            {
                                openingsInThisWall++;
                                openingsProcessed++;
                                
                                // Проверяем, пересекается ли окно/дверь с внешней стеной по высоте
                                BoundingBoxXYZ openingBbox = opening.get_BoundingBox(null);
                                BoundingBoxXYZ externalBbox2 = externalWall.get_BoundingBox(null);
                                
                                if (openingBbox != null && externalBbox2 != null)
                                {
                                    bool zOverlap = openingBbox.Max.Z >= externalBbox2.Min.Z && openingBbox.Min.Z <= externalBbox2.Max.Z;
                                    
                                    if (zOverlap)
                                    {
                                        Log(doc, $"  → Окно/дверь {opening.Id} должно прорезать внешнюю стену {externalWall.Id} " +
                                            $"(высота окна: {openingBbox.Min.Z:F3}-{openingBbox.Max.Z:F3}, " +
                                            $"высота стены: {externalBbox2.Min.Z:F3}-{externalBbox2.Max.Z:F3})");
                                        
                                        // Дополнительная попытка: используем CutGeometry для явного создания проема
                                        // Это может помочь, если Join Geometry не создает проемы автоматически
                                        try
                                        {
                                            // Пробуем использовать CutGeometry между окном/дверью и внешней стеной
                                            // Но это может не работать, так как окно уже привязано к внутренней стене
                                            // Альтернатива: проверить, есть ли уже проем во внешней стене
                                            // Если нет - возможно, нужно создать его вручную
                                            
                                            // Проверяем, есть ли проемы во внешней стене
                                            FilteredElementCollector openingCollector = new FilteredElementCollector(doc)
                                                .OfClass(typeof(Opening))
                                                .WherePasses(new ElementIntersectsElementFilter(externalWall));
                                            
                                            int openingCount = openingCollector.GetElementCount();
                                            Log(doc, $"    Проверка: найдено {openingCount} проемов во внешней стене {externalWall.Id}");
                                            
                                            if (openingCount == 0)
                                            {
                                                Log(doc, $"    ⚠ ВНИМАНИЕ: Во внешней стене {externalWall.Id} нет проемов, хотя окно/дверь {opening.Id} должно их создавать!");
                                                Log(doc, $"    Возможно, требуется ручное создание проема или использование другого метода.");
                                            }
                                        }
                                        catch (Exception cutEx)
                                        {
                                            Log(doc, $"    Ошибка при проверке проемов: {cutEx.Message}");
                                        }
                                    }
                                    else
                                    {
                                        Log(doc, $"  ⚠ Окно/дверь {opening.Id} НЕ пересекается с внешней стеной {externalWall.Id} по высоте! " +
                                            $"Окно: {openingBbox.Min.Z:F3}-{openingBbox.Max.Z:F3}, " +
                                            $"Стена: {externalBbox2.Min.Z:F3}-{externalBbox2.Max.Z:F3}");
                                    }
                                }
                            }
                        }
                        
                        if (openingsInThisWall == 0)
                        {
                            Log(doc, $"  → В стене {innerWall.Id} нет окон/дверей для помещения {room.Id}");
                        }
                    }
                    catch (Exception ex)
                    {
                        Log(doc, $"Ошибка при соединении геометрии стен {innerWall.Id} и {externalWall.Id}: {ex.Message}");
                        Log(doc, $"StackTrace: {ex.StackTrace}");
                    }
                    } // Конец цикла foreach (Wall externalWall in externalWalls)
                } // Конец цикла foreach (var kvp in innerWallRoomToExternalWallsMap)

                // Regenerate после соединения, чтобы проемы появились
                doc.Regenerate();

                // Дополнительная проверка: убеждаемся, что проемы действительно прорезают внешние стены
                // Для этого проверяем геометрию окон/дверей после соединения
                foreach (FamilyInstance opening in openings)
                {
                    Wall hostWall = opening.Host as Wall;
                    if (hostWall == null) continue;

                    // Ищем внешние стены для этой внутренней стены и текущего помещения
                    Tuple<ElementId, ElementId> key = new Tuple<ElementId, ElementId>(hostWall.Id, room.Id);
                    if (innerWallRoomToExternalWallsMap.TryGetValue(key, out List<Wall> externalWallsForThisPair))
                    {
                        // Проверяем все внешние стены для этой пары
                        foreach (Wall externalWall in externalWallsForThisPair)
                        {
                            // Проверяем, что стены соединены
                            if (JoinGeometryUtils.AreElementsJoined(doc, hostWall, externalWall))
                            {
                                // Получаем геометрию окна/двери для проверки проема
                                Options options = new Options();
                                options.ComputeReferences = true;
                                options.DetailLevel = ViewDetailLevel.Fine;
                                
                                try
                                {
                                    GeometryElement geom = opening.get_Geometry(options);
                                    if (geom != null)
                                    {
                                        // Проверяем, есть ли void геометрия, которая должна резать стены
                                        foreach (GeometryObject obj in geom)
                                        {
                                            if (obj is GeometryInstance inst)
                                            {
                                                GeometryElement instGeom = inst.GetInstanceGeometry();
                                                if (instGeom != null)
                                                {
                                                    foreach (GeometryObject instObj in instGeom)
                                                    {
                                                        if (instObj is Solid solid && solid.Volume > 0)
                                                        {
                                                            Log(doc, $"Найдена геометрия окна/двери {opening.Id} - должна прорезать стены (внешняя стена {externalWall.Id})");
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                                catch (Exception ex)
                                {
                                    Log(doc, $"Ошибка при проверке геометрии окна/двери {opening.Id}: {ex.Message}");
                                }
                            }
                        } // Конец цикла foreach (Wall externalWall in externalWallsForThisPair)
                    }
                }

                if (joinsCreated > 0)
                {
                    Log(doc, $"Соединена геометрия для {joinsCreated} пар стен. Обработано {openingsProcessed} окон/дверей из {openings.Count}.");
                    Log(doc, $"Проемы должны автоматически прорезать внешние стены благодаря соединению геометрии.");
                }
                else
                {
                    Log(doc, $"ВНИМАНИЕ: Не удалось соединить геометрию стен. Проемы могут не создаться автоматически.");
                }
            }
            catch (Exception ex)
            {
                Log(doc, $"Ошибка при соединении геометрии стен: {ex.Message}");
                Log(doc, $"StackTrace: {ex.StackTrace}");
            }

            return joinsCreated;
        }

        /// <summary>
        /// Устанавливает окна и двери во внешние стены с тем же семейством и типоразмером, что и в исходных стенах
        /// Не копирует параметры - просто устанавливает то же семейство и типоразмер
        /// </summary>
        private static int CopyOpeningsToExternalWalls(Document doc, List<Room> rooms, Dictionary<Tuple<ElementId, ElementId>, List<Wall>> innerWallRoomToExternalWallsMap)
        {
            int copiedCount = 0;

            try
            {
                doc.Regenerate(); // Убеждаемся, что все стены созданы

                // Множество для отслеживания уже обработанных окон/дверей
                // Каждое окно/дверь устанавливается только один раз
                HashSet<ElementId> copiedOpenings = new HashSet<ElementId>();

                foreach (Room room in rooms)
                {
                    // Получаем окна и двери из помещения
                    List<FamilyInstance> openings = GetWindowsAndDoorsFromRoom(doc, room);

                    foreach (FamilyInstance opening in openings)
                    {
                        Wall hostWall = opening.Host as Wall;
                        if (hostWall == null) continue;

                        // Проверяем, не было ли уже скопировано это окно/дверь
                        // Если окно уже скопировано, пропускаем его, чтобы избежать дубликатов
                        if (copiedOpenings.Contains(opening.Id))
                        {
                            continue; // Уже скопировано, пропускаем
                        }

                        // Ищем внешние стены для этой внутренней стены и помещения
                        Tuple<ElementId, ElementId> key = new Tuple<ElementId, ElementId>(hostWall.Id, room.Id);
                        if (!innerWallRoomToExternalWallsMap.TryGetValue(key, out List<Wall> externalWalls))
                        {
                            continue;
                        }

                        if (externalWalls == null || externalWalls.Count == 0)
                        {
                            continue;
                        }

                        // Получаем параметры исходного окна/двери
                        FamilySymbol familySymbol = opening.Symbol;
                        if (familySymbol == null) continue;

                        // Активируем семейство, если оно не активно
                        if (!familySymbol.IsActive)
                        {
                            try
                            {
                                familySymbol.Activate();
                                doc.Regenerate();
                            }
                            catch (Exception ex)
                            {
                                Log(doc, $"Не удалось активировать семейство {familySymbol.Name}: {ex.Message}");
                                continue;
                            }
                        }

                        // Получаем уровень
                        Level level = doc.GetElement(opening.LevelId) as Level;
                        if (level == null)
                        {
                            level = GetWallLevel(hostWall);
                        }
                        if (level == null) continue;

                        // Получаем реальную позицию окна/двери в пространстве (не относительно оси стены)
                        LocationPoint locationPoint = opening.Location as LocationPoint;
                        LocationCurve locationCurve = opening.Location as LocationCurve;

                        XYZ openingAbsolutePosition = null;
                        if (locationPoint != null)
                        {
                            openingAbsolutePosition = locationPoint.Point;
                        }
                        else if (locationCurve != null && locationCurve.Curve != null)
                        {
                            // Для LocationCurve берем среднюю точку кривой окна
                            openingAbsolutePosition = locationCurve.Curve.Evaluate(0.5, true);
                        }

                        if (openingAbsolutePosition == null) continue;

                        // Находим ближайшую внешнюю стену к реальной позиции окна
                        // Это важно, когда стена разделена на сегменты для разных помещений
                        Wall bestExternalWall = null;
                        double minDistance = double.MaxValue;
                        XYZ bestInsertionPoint = null;

                        foreach (Wall candidateWall in externalWalls)
                        {
                            LocationCurve candidateWallLocation = candidateWall.Location as LocationCurve;
                            if (candidateWallLocation == null || candidateWallLocation.Curve == null)
                                continue;

                            Curve candidateCurve = candidateWallLocation.Curve;

                            try
                            {
                                // Проецируем абсолютную позицию окна на кривую внешней стены
                                // Это даст точку на оси внешней стены, ближайшую к реальной позиции окна
                                IntersectionResult proj = candidateCurve.Project(openingAbsolutePosition);
                                double param = proj.Parameter;
                                
                                // ВАЖНО: false - param это натуральный параметр (не нормализованный 0..1)
                                XYZ candidatePointOnCurve = candidateCurve.Evaluate(param, false);
                                
                                // Сохраняем Z-координату (высоту) из исходного окна
                                XYZ candidatePoint = new XYZ(
                                    candidatePointOnCurve.X,
                                    candidatePointOnCurve.Y,
                                    openingAbsolutePosition.Z);
                                
                                // Вычисляем расстояние от реальной позиции окна до оси внешней стены
                                double distance = openingAbsolutePosition.DistanceTo(candidatePoint);

                                // Если эта стена ближе к окну, используем её
                                if (distance < minDistance)
                                {
                                    minDistance = distance;
                                    bestExternalWall = candidateWall;
                                    bestInsertionPoint = candidatePoint;
                                }
                            }
                            catch
                            {
                                // Если проекция не удалась, пробуем найти ближайшую точку
                                for (int i = 0; i <= 100; i++)
                                {
                                    double t = i / 100.0;
                                    XYZ testPointOnCurve = candidateCurve.Evaluate(t, true);
                                    // Сохраняем Z-координату из исходного окна
                                    XYZ testPoint = new XYZ(
                                        testPointOnCurve.X,
                                        testPointOnCurve.Y,
                                        openingAbsolutePosition.Z);
                                    double distance = openingAbsolutePosition.DistanceTo(testPoint);
                                    if (distance < minDistance)
                                    {
                                        minDistance = distance;
                                        bestExternalWall = candidateWall;
                                        bestInsertionPoint = testPoint;
                                    }
                                }
                            }
                        }

                        // Если не нашли подходящую стену, пропускаем
                        if (bestExternalWall == null || bestInsertionPoint == null)
                        {
                            continue;
                        }

                        try
                        {
                            FamilyInstance newOpening = null;
                            XYZ insertionPoint = bestInsertionPoint;

                            if (insertionPoint != null)
                            {
                                // Создаем новое окно/дверь на внешней стене с тем же семейством и типоразмером
                                newOpening = doc.Create.NewFamilyInstance(
                                    insertionPoint,
                                    familySymbol,
                                    bestExternalWall,
                                    level,
                                    Autodesk.Revit.DB.Structure.StructuralType.NonStructural);

                                if (newOpening != null)
                                {
                                    // Копируем параметр высоты подоконника из исходного окна
                                    // чтобы сохранить ту же высоту размещения
                                    Parameter sourceSillHeight = opening.get_Parameter(BuiltInParameter.INSTANCE_SILL_HEIGHT_PARAM);
                                    if (sourceSillHeight != null && sourceSillHeight.HasValue)
                                    {
                                        Parameter targetSillHeight = newOpening.get_Parameter(BuiltInParameter.INSTANCE_SILL_HEIGHT_PARAM);
                                        if (targetSillHeight != null && !targetSillHeight.IsReadOnly)
                                        {
                                            try
                                            {
                                                targetSillHeight.Set(sourceSillHeight.AsDouble());
                                                Log(doc, $"Скопирована высота подоконника: {sourceSillHeight.AsDouble():F3}");
                                            }
                                            catch (Exception ex)
                                            {
                                                Log(doc, $"Не удалось скопировать высоту подоконника: {ex.Message}");
                                            }
                                        }
                                    }

                                    // Копируем параметры ширины и высоты окна из исходного окна
                                    CopyWindowDimensions(doc, opening, newOpening);

                                    // Устанавливаем параметр ADSK_Зона для окна/двери
                                    SetZoneParameter(doc, newOpening, bestExternalWall);

                                    copiedCount++;
                                    Log(doc, $"Установлено окно/дверь {familySymbol.Name} во внешнюю стену {bestExternalWall.Id} (новый ID: {newOpening.Id}, расстояние до исходной позиции: {minDistance:F3})");
                                    
                                    // Помечаем это окно/дверь как обработанное
                                    // Это предотвратит дублирование, если стена граничит с несколькими помещениями
                                    copiedOpenings.Add(opening.Id);
                                }
                            }
                        }
                        catch (Exception ex)
                        {
                            Log(doc, $"Ошибка при установке окна/двери {opening.Id}: {ex.Message}");
                        }
                    }
                }

                doc.Regenerate();
            }
            catch (Exception ex)
            {
                Log(doc, $"Ошибка при установке окон/дверей во внешние стены: {ex.Message}");
            }

            return copiedCount;
        }

        /// <summary>
        /// Копирует параметры ширины и высоты окна/двери из исходного экземпляра в новый
        /// Также проверяет размеры через BoundingBox для дополнительной валидации
        /// </summary>
        private static void CopyWindowDimensions(Document doc, FamilyInstance source, FamilyInstance target)
        {
            try
            {
                // Получаем размеры из BoundingBox исходного окна
                BoundingBoxXYZ sourceBbox = source.get_BoundingBox(null);
                double sourceWidth = 0;
                double sourceHeight = 0;
                
                if (sourceBbox != null)
                {
                    // Вычисляем ширину и высоту из BoundingBox
                    // Ширина - это размер вдоль стены (в плоскости XY)
                    // Высота - это размер по вертикали (Z)
                    LocationCurve sourceLocation = source.Location as LocationCurve;
                    if (sourceLocation != null && sourceLocation.Curve != null)
                    {
                        // Для окон с LocationCurve ширина определяется по длине кривой
                        sourceWidth = sourceLocation.Curve.Length;
                    }
                    else
                    {
                        // Для окон с LocationPoint ширина определяется из BoundingBox
                        double dx = sourceBbox.Max.X - sourceBbox.Min.X;
                        double dy = sourceBbox.Max.Y - sourceBbox.Min.Y;
                        sourceWidth = Math.Max(dx, dy);
                    }
                    sourceHeight = sourceBbox.Max.Z - sourceBbox.Min.Z;
                }

                // Пытаемся скопировать параметры ширины и высоты
                // Сначала пробуем BuiltInParameter, затем имена параметров
                bool widthCopied = false;
                bool heightCopied = false;

                // Пробуем скопировать ширину через BuiltInParameter
                try
                {
                    Parameter sourceWidthParam = source.get_Parameter(BuiltInParameter.FAMILY_WIDTH_PARAM);
                    if (sourceWidthParam != null && sourceWidthParam.HasValue && !sourceWidthParam.IsReadOnly)
                    {
                        Parameter targetWidthParam = target.get_Parameter(BuiltInParameter.FAMILY_WIDTH_PARAM);
                        if (targetWidthParam != null && !targetWidthParam.IsReadOnly && sourceWidthParam.StorageType == StorageType.Double)
                        {
                            double widthValue = sourceWidthParam.AsDouble();
                            targetWidthParam.Set(widthValue);
                            widthCopied = true;
                            Log(doc, $"Скопирована ширина окна: {widthValue:F3} (BuiltInParameter.FAMILY_WIDTH_PARAM)");
                        }
                    }
                }
                catch { }

                // Если не получилось через BuiltInParameter, пробуем по именам
                if (!widthCopied)
                {
                    string[] widthParamNames = { "Width", "Ширина", "Default Width", "Ширина по умолчанию" };
                    foreach (string paramName in widthParamNames)
                    {
                        Parameter sourceWidthParam = source.LookupParameter(paramName);
                        if (sourceWidthParam != null && sourceWidthParam.HasValue && !sourceWidthParam.IsReadOnly)
                        {
                            Parameter targetWidthParam = target.LookupParameter(paramName);
                            if (targetWidthParam != null && !targetWidthParam.IsReadOnly)
                            {
                                try
                                {
                                    if (sourceWidthParam.StorageType == StorageType.Double)
                                    {
                                        double widthValue = sourceWidthParam.AsDouble();
                                        targetWidthParam.Set(widthValue);
                                        widthCopied = true;
                                        Log(doc, $"Скопирована ширина окна: {widthValue:F3} (параметр: {paramName})");
                                        break;
                                    }
                                }
                                catch (Exception ex)
                                {
                                    Log(doc, $"Не удалось скопировать ширину через параметр {paramName}: {ex.Message}");
                                }
                            }
                        }
                    }
                }

                // Пробуем скопировать высоту через BuiltInParameter
                try
                {
                    Parameter sourceHeightParam = source.get_Parameter(BuiltInParameter.FAMILY_HEIGHT_PARAM);
                    if (sourceHeightParam != null && sourceHeightParam.HasValue && !sourceHeightParam.IsReadOnly)
                    {
                        Parameter targetHeightParam = target.get_Parameter(BuiltInParameter.FAMILY_HEIGHT_PARAM);
                        if (targetHeightParam != null && !targetHeightParam.IsReadOnly && sourceHeightParam.StorageType == StorageType.Double)
                        {
                            double heightValue = sourceHeightParam.AsDouble();
                            targetHeightParam.Set(heightValue);
                            heightCopied = true;
                            Log(doc, $"Скопирована высота окна: {heightValue:F3} (BuiltInParameter.FAMILY_HEIGHT_PARAM)");
                        }
                    }
                }
                catch { }

                // Если не получилось через BuiltInParameter, пробуем по именам
                if (!heightCopied)
                {
                    string[] heightParamNames = { "Height", "Высота", "Default Height", "Высота по умолчанию" };
                    foreach (string paramName in heightParamNames)
                    {
                        Parameter sourceHeightParam = source.LookupParameter(paramName);
                        if (sourceHeightParam != null && sourceHeightParam.HasValue && !sourceHeightParam.IsReadOnly)
                        {
                            Parameter targetHeightParam = target.LookupParameter(paramName);
                            if (targetHeightParam != null && !targetHeightParam.IsReadOnly)
                            {
                                try
                                {
                                    if (sourceHeightParam.StorageType == StorageType.Double)
                                    {
                                        double heightValue = sourceHeightParam.AsDouble();
                                        targetHeightParam.Set(heightValue);
                                        heightCopied = true;
                                        Log(doc, $"Скопирована высота окна: {heightValue:F3} (параметр: {paramName})");
                                        break;
                                    }
                                }
                                catch (Exception ex)
                                {
                                    Log(doc, $"Не удалось скопировать высоту через параметр {paramName}: {ex.Message}");
                                }
                            }
                        }
                    }
                }

                // Проверяем размеры через BoundingBox после копирования параметров
                doc.Regenerate();
                BoundingBoxXYZ targetBbox = target.get_BoundingBox(null);
                
                if (sourceBbox != null && targetBbox != null)
                {
                    double targetWidth = 0;
                    double targetHeight = 0;
                    
                    LocationCurve targetLocation = target.Location as LocationCurve;
                    if (targetLocation != null && targetLocation.Curve != null)
                    {
                        targetWidth = targetLocation.Curve.Length;
                    }
                    else
                    {
                        double dx = targetBbox.Max.X - targetBbox.Min.X;
                        double dy = targetBbox.Max.Y - targetBbox.Min.Y;
                        targetWidth = Math.Max(dx, dy);
                    }
                    targetHeight = targetBbox.Max.Z - targetBbox.Min.Z;

                    // Проверяем совпадение размеров с допуском 1 мм (0.003 фута)
                    const double tolerance = 0.003;
                    bool widthMatches = Math.Abs(sourceWidth - targetWidth) < tolerance;
                    bool heightMatches = Math.Abs(sourceHeight - targetHeight) < tolerance;

                    if (!widthMatches || !heightMatches)
                    {
                        Log(doc, $"⚠️ Размеры окна не совпадают! Исходное: ширина={sourceWidth:F3}, высота={sourceHeight:F3}; " +
                            $"Новое: ширина={targetWidth:F3}, высота={targetHeight:F3}");
                        
                        if (!widthCopied)
                        {
                            Log(doc, $"Ширина не была скопирована через параметры. Разница: {Math.Abs(sourceWidth - targetWidth):F3}");
                        }
                        if (!heightCopied)
                        {
                            Log(doc, $"Высота не была скопирована через параметры. Разница: {Math.Abs(sourceHeight - targetHeight):F3}");
                        }
                    }
                    else
                    {
                        Log(doc, $"✓ Размеры окна совпадают: ширина={targetWidth:F3}, высота={targetHeight:F3}");
                    }
                }
            }
            catch (Exception ex)
            {
                Log(doc, $"Ошибка при копировании размеров окна: {ex.Message}");
            }
        }

        /// <summary>
        /// Копирует параметры экземпляра из одного FamilyInstance в другой
        /// </summary>
        private static void CopyInstanceParameters(FamilyInstance source, FamilyInstance target)
        {
            try
            {
                foreach (Parameter sourceParam in source.Parameters)
                {
                    if (sourceParam.IsReadOnly || !sourceParam.HasValue) continue;

                    Parameter targetParam = target.LookupParameter(sourceParam.Definition.Name);
                    if (targetParam == null || targetParam.IsReadOnly) continue;

                    try
                    {
                        switch (sourceParam.StorageType)
                        {
                            case StorageType.Double:
                                targetParam.Set(sourceParam.AsDouble());
                                break;
                            case StorageType.Integer:
                                targetParam.Set(sourceParam.AsInteger());
                                break;
                            case StorageType.String:
                                targetParam.Set(sourceParam.AsString());
                                break;
                            case StorageType.ElementId:
                                ElementId id = sourceParam.AsElementId();
                                if (id != null && id != ElementId.InvalidElementId)
                                {
                                    targetParam.Set(id);
                                }
                                break;
                        }
                    }
                    catch
                    {
                        // Игнорируем ошибки при копировании отдельных параметров
                    }
                }
            }
            catch (Exception ex)
            {
                // Логируем ошибку, но не прерываем выполнение
            }
        }

        /// <summary>
        /// Определяет сторону света (Ю, С, В, З) по направлению нормали внешней стены
        /// </summary>
        private static string GetCardinalDirection(Wall wall)
        {
            try
            {
                if (wall == null)
                    return "С";

                // Получаем нормаль к внешней стороне стены
                // Используем среднюю точку стены для более точного определения направления
                LocationCurve locationCurve = wall.Location as LocationCurve;
                if (locationCurve == null || locationCurve.Curve == null)
                    return "С";

                Curve curve = locationCurve.Curve;
                XYZ midPoint = curve.Evaluate(0.5, true);

                // Получаем внешнюю грань стены для более точного определения нормали
                XYZ normal = null;
                try
                {
                    var sideRefs = HostObjectUtils.GetSideFaces(wall, ShellLayerType.Exterior);
                    if (sideRefs != null && sideRefs.Count > 0)
                    {
                        Reference firstRef = sideRefs[0];
                        Face face = wall.Document.GetElement(firstRef)?.GetGeometryObjectFromReference(firstRef) as Face;
                        if (face != null)
                        {
                            // Получаем нормаль в средней точке грани
                            UV uv = face.Project(midPoint).UVPoint;
                            normal = face.ComputeNormal(uv);
                            // Проецируем на горизонтальную плоскость (XY)
                            normal = new XYZ(normal.X, normal.Y, 0).Normalize();
                            // Инвертируем нормаль, так как ComputeNormal может указывать внутрь
                            // Нам нужно направление наружу от здания
                            normal = -normal;
                        }
                    }
                }
                catch
                {
                    // Если не удалось получить грань, используем метод GetWallFaceNormal
                }

                // Если не удалось получить нормаль из грани, используем упрощенный метод
                if (normal == null || normal.GetLength() < 0.1)
                {
                    normal = GetWallFaceNormal(wall);
                    // Инвертируем нормаль, так как GetWallFaceNormal может указывать в неправильном направлении
                    normal = -normal;
                }

                // Вычисляем азимут (угол от севера)
                // В Revit: Y - север, X - восток
                // atan2(X, Y) дает угол от севера по часовой стрелке
                double azimuth = Math.Atan2(normal.X, normal.Y) * 180.0 / Math.PI;
                
                // Нормализуем азимут в диапазон 0-360
                if (azimuth < 0)
                    azimuth += 360.0;
                
                // Определяем сторону света:
                // Север: 315-45 градусов -> С
                // Восток: 45-135 градусов -> В
                // Юг: 135-225 градусов -> Ю
                // Запад: 225-315 градусов -> З
                
                if (azimuth >= 315.0 || azimuth < 45.0)
                    return "С"; // Север
                else if (azimuth >= 45.0 && azimuth < 135.0)
                    return "В"; // Восток
                else if (azimuth >= 135.0 && azimuth < 225.0)
                    return "Ю"; // Юг
                else // azimuth >= 225.0 && azimuth < 315.0
                    return "З"; // Запад
            }
            catch
            {
                return "С"; // По умолчанию Север
            }
        }

        /// <summary>
        /// Устанавливает параметр ADSK_Зона для стены, окна или двери
        /// </summary>
        private static void SetZoneParameter(Document doc, Element element, Wall hostWall)
        {
            try
            {
                if (element == null)
                    return;

                // Определяем сторону света
                // Для окон и дверей используем направление стены-хозяина
                Wall wallToUse = hostWall ?? (element as Wall);
                if (wallToUse == null)
                    return;

                string direction = GetCardinalDirection(wallToUse);
                
                // Ищем параметр ADSK_Зона
                Parameter zoneParam = element.LookupParameter("ADSK_Зона");
                if (zoneParam == null)
                {
                    // Пробуем альтернативные варианты названия
                    foreach (Parameter param in element.Parameters)
                    {
                        if (param.Definition != null && 
                            (param.Definition.Name == "ADSK_Зона" ||
                             param.Definition.Name.Contains("Зона") || 
                             param.Definition.Name.Contains("Zone")))
                        {
                            zoneParam = param;
                            break;
                        }
                    }
                }

                if (zoneParam != null && !zoneParam.IsReadOnly)
                {
                    if (zoneParam.StorageType == StorageType.String)
                    {
                        zoneParam.Set(direction);
                        Log(doc, $"Установлен параметр ADSK_Зона = {direction} для элемента {element.Id} (тип: {element.GetType().Name})");
                    }
                    else
                    {
                        Log(doc, $"Параметр ADSK_Зона найден, но имеет неподдерживаемый тип хранения {zoneParam.StorageType} для элемента {element.Id}");
                    }
                }
                else
                {
                    Log(doc, $"Параметр ADSK_Зона не найден или только для чтения для элемента {element.Id} (тип: {element.GetType().Name})");
                }
            }
            catch (Exception ex)
            {
                Log(doc, $"Ошибка при установке параметра ADSK_Зона для элемента {element?.Id}: {ex.Message}");
            }
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
            // Список ID созданных стен для исключения из проверки пересечений
            HashSet<ElementId> createdWallIds = new HashSet<ElementId>();
            // Словарь соответствия внутренних и внешних стен для создания проемов
            // Ключ: Tuple<ElementId внутренней стены, ElementId помещения>
            // Значение: Список внешних стен (может быть несколько, если стена разделена на сегменты)
            Dictionary<Tuple<ElementId, ElementId>, List<Wall>> innerWallRoomToExternalWallsMap = 
                new Dictionary<Tuple<ElementId, ElementId>, List<Wall>>();

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
                    // Проверяем, является ли стена внешней (хотя бы с одной стороны нет помещения)
                    // Создаем список boundary segments для этой стены
                    List<BoundarySegmentData> segmentDataList = new List<BoundarySegmentData>();
                    
                    // Находим boundary segment для этого помещения
                    foreach (IList<BoundarySegment> loop in boundaryLoops)
                    {
                        foreach (BoundarySegment segment in loop)
                        {
                            if (segment.ElementId == innerWall.Id)
                            {
                                segmentDataList.Add(new BoundarySegmentData
                                {
                                    Segment = segment,
                                    Room = room,
                                    Curve = segment.GetCurve()
                                });
                            }
                        }
                    }

                    if (!IsExternalWall(doc, innerWall, segmentDataList))
                    {
                        Log(doc, $"Стена {innerWall.Id} имеет помещения с обеих сторон, пропуск (перегородка)");
                        continue;
                    }

                    // Осевая линия существующей стены (её расчётная длина)
                    LocationCurve wallLocation = innerWall.Location as LocationCurve;
                    if (wallLocation == null || wallLocation.Curve == null || !(wallLocation.Curve is Line axisLine))
                    {
                        Log(doc, $"Стена {innerWall.Id} не имеет подходящей осевой линии");
                        continue;
                    }

                    Log(doc, $"Обрабатываем стену {innerWall.Id}, длина оси: {axisLine.Length:F3}");

                    // Получаем параметры стены
                    Level level = GetWallLevel(innerWall);
                    double height = GetWallHeight(innerWall);
                    if (level == null)
                    {
                        Log(doc, $"Не удалось получить уровень для стены {innerWall.Id}");
                        continue;
                    }

                    // Вычисляем смещение от оси внутренней стены до оси внешней:
                    // берём половину толщины внутренней + половину толщины новой стены.
                    double innerThickness = GetWallThickness(innerWall);
                    double externalThickness = GetWallTypeThickness(wallType);
                    double offsetDistance = (innerThickness / 2.0) + (externalThickness / 2.0);

                    Log(doc, $"Смещение: {offsetDistance:F3} (толщина внутренней: {innerThickness:F3}, внешней: {externalThickness:F3})");

                    // Определяем направление наружу от помещения
                    // Используем граничную кривую для определения направления
                    Curve boundaryCurve = null;
                    wallBoundaryCurves.TryGetValue(innerWall.Id, out boundaryCurve);

                    XYZ outwardNormal = GetOutwardNormalFromRoom(innerWall, boundaryCurve ?? axisLine, room);

                    // Строим ось внешней стены: ось внутренней + смещение по нормали
                    XYZ axisStart = axisLine.GetEndPoint(0);
                    XYZ axisEnd   = axisLine.GetEndPoint(1);
                    XYZ externalStart = axisStart + outwardNormal * offsetDistance;
                    XYZ externalEnd   = axisEnd   + outwardNormal * offsetDistance;

                    Curve externalCurve = Line.CreateBound(externalStart, externalEnd);
                    
                    // Проверяем, не пересекает ли создаваемая стена комнату
                    // Если пересекает, пробуем противоположное направление
                    // Передаем boundary curve для более точной проверки
                    if (DoesWallIntersectRoom(externalCurve, innerWall, room, boundaryCurve))
                    {
                        // Пробуем противоположное направление
                        XYZ oppositeNormal = -outwardNormal;
                        XYZ oppositeStart = axisStart + oppositeNormal * offsetDistance;
                        XYZ oppositeEnd = axisEnd + oppositeNormal * offsetDistance;
                        Curve oppositeCurve = Line.CreateBound(oppositeStart, oppositeEnd);
                        
                        // Если противоположное направление не пересекает комнату, используем его
                        if (!DoesWallIntersectRoom(oppositeCurve, innerWall, room, boundaryCurve))
                        {
                            outwardNormal = oppositeNormal;
                            externalStart = oppositeStart;
                            externalEnd = oppositeEnd;
                            externalCurve = oppositeCurve;
                        }
                        // Иначе оставляем исходное направление (может быть случай, когда оба пересекают)
                        else
                        {
                            outwardNormal = -outwardNormal;
                            externalStart = axisStart + outwardNormal * offsetDistance;
                            externalEnd = axisEnd + outwardNormal * offsetDistance;
                            externalCurve = Line.CreateBound(externalStart, externalEnd);
                        }
                    }
                    
                    // Дотягиваем до торцов исходной стены (учёт её толщины и примыканий)
                    externalCurve = ExtendToWallEnds(innerWall, externalCurve);
                    externalCurve = ExtendCurveToJoinedWalls(innerWall, externalCurve);

                    if (externalCurve == null || externalCurve.Length < 0.01)
                    {
                        Log(doc, $"Смещенная кривая слишком короткая для стены {innerWall.Id}");
                        continue;
                    }

                    // Проверяем пересечение с существующими стенами и обрезаем при необходимости
                    // Исключаем из проверки уже созданные в этой транзакции стены
                    externalCurve = TrimCurveAgainstExistingWalls(doc, externalCurve, innerWall, createdWallIds);
                    if (externalCurve == null || externalCurve.Length < 0.01)
                    {
                        Log(doc, $"Кривая обрезана до нуля для стены {innerWall.Id}");
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
                        // НЕ отключаем соединения стен здесь - это может помешать Join Geometry
                        // DisableWallJoins будет вызван после соединения геометрии, если нужно
                        // Копируем свойства из внутренней стены
                        CopyWallProperties(innerWall, externalWall);
                        
                        // Устанавливаем параметр ADSK_Зона для внешней стены
                        SetZoneParameter(doc, externalWall, null);
                        
                        created++;
                        // Добавляем ID созданной стены в список для исключения из проверки
                        createdWallIds.Add(externalWall.Id);
                        // Сохраняем соответствие между внутренней и внешней стеной с привязкой к помещению
                        // Может быть несколько внешних стен для одной пары (стена, помещение), если стена разделена на сегменты
                        Tuple<ElementId, ElementId> key = new Tuple<ElementId, ElementId>(innerWall.Id, room.Id);
                        if (!innerWallRoomToExternalWallsMap.ContainsKey(key))
                        {
                            innerWallRoomToExternalWallsMap[key] = new List<Wall>();
                        }
                        innerWallRoomToExternalWallsMap[key].Add(externalWall);
                        Log(doc, $"Создана внешняя стена {externalWall.Id} для внутренней стены {innerWall.Id} помещения {room.Id} (всего внешних стен для этой пары: {innerWallRoomToExternalWallsMap[key].Count})");
                    }
                    else
                    {
                        Log(doc, $"Не удалось создать внешнюю стену для {innerWall.Id}");
                    }
                }

                // Соединяем геометрию между внутренними и внешними стенами для автоматического создания проемов
                // Проверяем настройки: создавать ли проемы
                Settings settings = Settings.Load();
                if (settings.CreateOpenings)
                {
                    int joinsCreated = JoinGeometryBetweenWalls(doc, room, innerWallRoomToExternalWallsMap);
                    Log(doc, $"Соединена геометрия для {joinsCreated} пар стен. Проемы будут созданы автоматически.");
                }
                else
                {
                    Log(doc, "Создание проемов отключено в настройках. Пропускаем соединение геометрии.");
                }

                // Устанавливаем окна и двери во внешние стены, если это включено в настройках
                if (settings.CopyOpeningsToExternalWalls)
                {
                    List<Room> singleRoomList = new List<Room> { room };
                    int copiedCount = CopyOpeningsToExternalWalls(doc, singleRoomList, innerWallRoomToExternalWallsMap);
                    if (copiedCount > 0)
                    {
                        Log(doc, $"Установлено {copiedCount} окон/дверей во внешние стены.");
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
