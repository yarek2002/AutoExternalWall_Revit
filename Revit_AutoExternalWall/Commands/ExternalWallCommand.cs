using Autodesk.Revit.Attributes;
using Autodesk.Revit.DB;
using Autodesk.Revit.UI;
using Autodesk.Revit.UI.Selection;
using Revit_AutoExternalWall.Utilities;
using Autodesk.Revit.DB.Architecture;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Revit_AutoExternalWall
{
    /// <summary>
    /// External command for placing external walls around selected walls
    /// </summary>
    [Transaction(TransactionMode.Manual)]
    [Regeneration(RegenerationOption.Manual)]
    public class ExternalWallCommand : IExternalCommand
    {
        public Autodesk.Revit.UI.Result Execute(ExternalCommandData commandData, ref string message, Autodesk.Revit.DB.ElementSet elements)
        {
            UIApplication uiApp = commandData.Application;
            UIDocument uiDoc = uiApp.ActiveUIDocument;
            Document doc = uiDoc.Document;

            try
            {
                // Let user select walls and/or rooms. Prompt if nothing selected.
                ICollection<ElementId> selectedIds = uiDoc.Selection.GetElementIds();

                if (selectedIds.Count == 0)
                {
                    TaskDialog taskDialog = new TaskDialog("Select Walls or Rooms")
                    {
                        MainInstruction = "Please select walls and/or rooms",
                        MainContent = "Select interior walls and/or rooms for which you want to place external walls."
                    };
                    taskDialog.Show();

                    try
                    {
                        // Allow user to pick multiple elements (walls and rooms)
                        var refs = uiDoc.Selection.PickObjects(ObjectType.Element);
                        selectedIds = refs.Select(r => r.ElementId).ToList();
                    }
                    catch (Autodesk.Revit.Exceptions.OperationCanceledException)
                    {
                        return Autodesk.Revit.UI.Result.Cancelled;
                    }
                }

                if (selectedIds.Count == 0)
                {
                    TaskDialog.Show("No Selection", "No elements selected.");
                    return Autodesk.Revit.UI.Result.Cancelled;
                }

                // Separate selected walls and rooms
                List<Wall> selectedWalls = new List<Wall>();
                List<Room> selectedRooms = new List<Room>();
                foreach (var id in selectedIds)
                {
                    Element el = doc.GetElement(id);
                    if (el is Wall w) selectedWalls.Add(w);
                    else if (el is Room r) selectedRooms.Add(r);
                }

                if (selectedWalls.Count == 0 && selectedRooms.Count == 0)
                {
                    TaskDialog.Show("Invalid Selection", "Please select at least one wall or one room.");
                    return Autodesk.Revit.UI.Result.Cancelled;
                }

                // Get wall type for external walls
                WallType externalWallType = WallUtilities.GetExternalWallType(doc);
                if (externalWallType == null)
                {
                    TaskDialog.Show("Error", "Could not find suitable wall type for external walls.");
                    return Result.Failed;
                }

                // Start transaction
                using (Transaction trans = new Transaction(doc, "Place External Walls"))
                {
                    trans.Start();

                    // Place external walls
                    int wallsCreated = 0;

                    // Create walls based on selected walls
                    foreach (Wall wall in selectedWalls)
                    {
                        try
                        {
                            wallsCreated += WallUtilities.CreateExternalWall(doc, wall, externalWallType);
                        }
                        catch (Exception ex)
                        {
                            message += $"Error processing wall: {ex.Message}\n";
                        }
                    }

                    // Create walls based on selected rooms (use room boundaries)
                    foreach (var se in selectedRooms)
                    {
                        try
                        {
                            if (se is Room room)
                            {
                                wallsCreated += WallUtilities.CreateExternalWallsFromRoom(doc, room, externalWallType);
                            }
                        }
                        catch (Exception ex)
                        {
                            message += $"Error processing room: {ex.Message}\n";
                        }
                    }

                    trans.Commit();

                    TaskDialog.Show("Success", $"Created {wallsCreated} external wall(s).");
                }

                return Autodesk.Revit.UI.Result.Succeeded;
            }
            catch (Exception ex)
            {
                message = $"Error: {ex.Message}\nStack Trace: {ex.StackTrace}";
                TaskDialog.Show("Error", message);
                return Autodesk.Revit.UI.Result.Failed;
            }
        }
    }

    /// <summary>
    /// Selection filter for walls only
    /// </summary>
    public class WallSelectionFilter : ISelectionFilter
    {
        public bool AllowElement(Element elem)
        {
            return elem is Wall;
        }

        public bool AllowReference(Reference reference, XYZ position)
        {
            return true;
        }
    }
}
