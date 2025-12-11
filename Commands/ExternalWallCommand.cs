using Autodesk.Revit.Attributes;
using Autodesk.Revit.DB;
using Autodesk.Revit.UI;
using Autodesk.Revit.UI.Selection;
using Revit_AutoExternalWall.Utilities;
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
        public Result Execute(ExternalCommandData commandData, ref string message, ElementSet elements)
        {
            UIApplication uiApp = commandData.Application;
            UIDocument uiDoc = uiApp.ActiveUIDocument;
            Document doc = uiDoc.Document;

            try
            {
                // Select walls
                ICollection<ElementId> selectedIds = uiDoc.Selection.GetElementIds();

                if (selectedIds.Count == 0)
                {
                    // Prompt user to select walls
                    TaskDialog taskDialog = new TaskDialog("Select Walls")
                    {
                        MainInstruction = "Please select walls",
                        MainContent = "Select the interior walls for which you want to place external walls."
                    };
                    taskDialog.Show();

                    // Use selection filter for walls
                    try
                    {
                        selectedIds = uiDoc.Selection.PickObjects(ObjectType.Element, new WallSelectionFilter())
                            .Select(x => x.ElementId)
                            .ToList();
                    }
                    catch (Autodesk.Revit.Exceptions.OperationCanceledException)
                    {
                        return Result.Cancelled;
                    }
                }

                if (selectedIds.Count == 0)
                {
                    TaskDialog.Show("No Selection", "No walls selected.");
                    return Result.Cancelled;
                }

                // Get selected walls
                List<Wall> selectedWalls = selectedIds
                    .Select(id => doc.GetElement(id) as Wall)
                    .Where(w => w != null)
                    .ToList();

                if (selectedWalls.Count == 0)
                {
                    TaskDialog.Show("Invalid Selection", "Please select at least one wall.");
                    return Result.Cancelled;
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

                    trans.Commit();

                    TaskDialog.Show("Success", $"Created {wallsCreated} external wall(s).");
                }

                return Result.Succeeded;
            }
            catch (Exception ex)
            {
                message = $"Error: {ex.Message}\nStack Trace: {ex.StackTrace}";
                TaskDialog.Show("Error", message);
                return Result.Failed;
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
