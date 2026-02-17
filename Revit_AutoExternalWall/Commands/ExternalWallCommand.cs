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
                    TaskDialog taskDialog = new TaskDialog("Выбор стен или помещений")
                    {
                        MainInstruction = "Выберите стены и помещения",
                        MainContent = "Выберите внутренние стены и/или помещения, для которых вы хотите создать внешние стены."
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
                    TaskDialog.Show("Нет выбора", "Элементы не выбраны.");
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
                    TaskDialog.Show("Неверный выбор", "Пожалуйста, выберите хотя бы одну стену или одно помещение.");
                    return Autodesk.Revit.UI.Result.Cancelled;
                }

                // Get wall type for external walls
                WallType externalWallType = WallUtilities.GetExternalWallType(doc);
                if (externalWallType == null)
                {
                    TaskDialog.Show("Ошибка", "Не удалось найти подходящий тип стены для внешних стен.");
                    return Result.Failed;
                }

                // Start transaction
                using (Transaction trans = new Transaction(doc, "Создание внешних стен"))
                {
                    trans.Start();

                    // Place external walls
                    int wallsCreated = 0;

                    // Создаем внешние стены для выбранных помещений
                    if (selectedRooms.Count > 0)
                    {
                        try
                        {
                            if (selectedRooms.Count == 1)
                            {
                                // Для одного помещения используем простую логику
                                wallsCreated += WallUtilities.CreateExternalWallsFromSingleRoom(doc, selectedRooms[0], externalWallType);
                            }
                            else
                            {
                                // Для нескольких помещений используем логику с разделением стен
                                wallsCreated += WallUtilities.CreateExternalWallsFromRooms(doc, selectedRooms, externalWallType);
                            }
                        }
                        catch (Exception ex)
                        {
                            message += $"Ошибка при обработке помещений: {ex.Message}\n";
                        }
                    }
                    else
                    {
                        TaskDialog.Show("Информация", "Пожалуйста, выберите помещение для создания внешних стен.");
                        return Result.Cancelled;
                    }

                    trans.Commit();

                    TaskDialog.Show("Успех", $"Создано {wallsCreated} внешних стен.");
                }

                return Autodesk.Revit.UI.Result.Succeeded;
            }
            catch (Exception ex)
            {
                message = $"Ошибка: {ex.Message}\nТрассировка стека: {ex.StackTrace}";
                TaskDialog.Show("Ошибка", message);
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
