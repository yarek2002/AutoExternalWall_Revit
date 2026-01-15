using Autodesk.Revit.Attributes;
using Autodesk.Revit.UI;
using System;
using System.Windows;

namespace Revit_AutoExternalWall
{
    /// <summary>
    /// Команда для открытия окна настроек
    /// </summary>
    [Transaction(TransactionMode.Manual)]
    [Regeneration(RegenerationOption.Manual)]
    public class SettingsCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData commandData, ref string message, Autodesk.Revit.DB.ElementSet elements)
        {
            try
            {
                // Загружаем текущие настройки
                Settings settings = Settings.Load();

                // Создаем и показываем окно настроек
                SettingsWindow settingsWindow = new SettingsWindow(settings);
                settingsWindow.ShowDialog();

                return Result.Succeeded;
            }
            catch (Exception ex)
            {
                message = $"Ошибка при открытии настроек: {ex.Message}";
                TaskDialog.Show("Ошибка", message);
                return Result.Failed;
            }
        }
    }
}

