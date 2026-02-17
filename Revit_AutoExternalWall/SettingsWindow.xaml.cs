using System.Windows;
using System.Linq;
using Autodesk.Revit.DB;

namespace Revit_AutoExternalWall
{
    /// <summary>
    /// Окно настроек плагина
    /// </summary>
    public partial class SettingsWindow : Window
    {
        public Settings Settings { get; private set; }
        private Document Document { get; set; }

        public SettingsWindow(Settings settings, Document doc)
        {
            InitializeComponent();
            Settings = settings;
            Document = doc;
            chkCreateOpenings.IsChecked = settings.CreateOpenings;
            chkCopyOpenings.IsChecked = settings.CopyOpeningsToExternalWalls;
            chkSetApartmentNumber.IsChecked = settings.SetApartmentNumber;
            chkSetZone.IsChecked = settings.SetZone;
            
            // Заполняем ComboBox типами стен
            LoadWallTypes();
        }

        private void LoadWallTypes()
        {
            try
            {
                // Получаем все типы стен (только Basic Walls)
                var wallTypes = new FilteredElementCollector(Document)
                    .OfClass(typeof(WallType))
                    .Cast<WallType>()
                    .Where(wt => wt.Kind == WallKind.Basic)
                    .OrderBy(wt => wt.Name)
                    .ToList();

                cmbWallType.ItemsSource = wallTypes;

                // Устанавливаем выбранный тип стены из настроек
                if (!string.IsNullOrEmpty(Settings.SelectedWallTypeId))
                {
                    if (int.TryParse(Settings.SelectedWallTypeId, out int wallTypeIdInt))
                    {
                        ElementId wallTypeId = new ElementId(wallTypeIdInt);
                        var selectedWallType = wallTypes.FirstOrDefault(wt => wt.Id == wallTypeId);
                        if (selectedWallType != null)
                        {
                            cmbWallType.SelectedItem = selectedWallType;
                        }
                    }
                }

                // Если ничего не выбрано, выбираем первый элемент
                if (cmbWallType.SelectedItem == null && wallTypes.Count > 0)
                {
                    cmbWallType.SelectedIndex = 0;
                }
            }
            catch
            {
                // В случае ошибки просто оставляем список пустым
            }
        }

        private void BtnOk_Click(object sender, RoutedEventArgs e)
        {
            Settings.CreateOpenings = chkCreateOpenings.IsChecked ?? true;
            Settings.CopyOpeningsToExternalWalls = chkCopyOpenings.IsChecked ?? false;
            Settings.SetApartmentNumber = chkSetApartmentNumber.IsChecked ?? true;
            Settings.SetZone = chkSetZone.IsChecked ?? true;
            
            // Сохраняем выбранный тип стены
            if (cmbWallType.SelectedItem is WallType selectedWallType)
            {
                Settings.SelectedWallTypeId = selectedWallType.Id.IntegerValue.ToString();
            }
            else
            {
                Settings.SelectedWallTypeId = null;
            }
            
            Settings.Save();
            DialogResult = true;
            Close();
        }

        private void BtnCancel_Click(object sender, RoutedEventArgs e)
        {
            DialogResult = false;
            Close();
        }
    }
}

