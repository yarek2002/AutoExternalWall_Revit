using Autodesk.Revit.ApplicationServices;
using Autodesk.Revit.Attributes;
using Autodesk.Revit.DB;
using Autodesk.Revit.UI;
using System;
using System.IO;
using System.Reflection;
using System.Windows.Media.Imaging;

namespace Revit_AutoExternalWall
{
    /// <summary>
    /// Revit external application for AutoExternalWall plugin
    /// </summary>
    [Regeneration(RegenerationOption.Manual)]
    [Transaction(TransactionMode.Manual)]
    public class App : IExternalApplication
    {
        /// <summary>
        /// Implement this method to execute some tasks when Revit starts.
        /// </summary>
        public Result OnStartup(UIControlledApplication application)
        {
            try
            {
                // Create ribbon tab
                string tabName = "Создание внешних стен";
                try
                {
                    application.CreateRibbonTab(tabName);
                }
                catch
                {
                    // Tab already exists
                }

                // Create ribbon panel
                RibbonPanel panel = application.CreateRibbonPanel(tabName, "Инструменты");

                // Get the assembly path for button icon
                string assemblyPath = Assembly.GetExecutingAssembly().Location;
                string imagePath = Path.Combine(Path.GetDirectoryName(assemblyPath), "Resources");

                // Create push button for external wall command
                PushButtonData externalWallButtonData = new PushButtonData(
                    "cmdExternalWall",
                    "Создать внешние\nстены",
                    assemblyPath,
                    "Revit_AutoExternalWall.ExternalWallCommand");

                PushButton externalWallButton = panel.AddItem(externalWallButtonData) as PushButton;
                externalWallButton.ToolTip = "Создать внешние стены вокруг выбранных внутренних стен";

                // Set button image if available
                try
                {
                    BitmapImage icon = new BitmapImage(new Uri(Path.Combine(imagePath, "ExternalWall_32.png")));
                    externalWallButton.LargeImage = icon;
                }
                catch
                {
                    // If image not found, continue without it
                }

                // Create push button for settings command
                PushButtonData settingsButtonData = new PushButtonData(
                    "cmdSettings",
                    "Настройки",
                    assemblyPath,
                    "Revit_AutoExternalWall.SettingsCommand");

                PushButton settingsButton = panel.AddItem(settingsButtonData) as PushButton;
                settingsButton.ToolTip = "Открыть настройки плагина";

                return Result.Succeeded;
            }
            catch (Exception ex)
            {
                TaskDialog.Show("Ошибка", $"Ошибка при запуске: {ex.Message}");
                return Result.Failed;
            }
        }

        /// <summary>
        /// Implement this method to execute some tasks when Revit shuts down.
        /// </summary>
        public Result OnShutdown(UIControlledApplication application)
        {
            return Result.Succeeded;
        }
    }
}
