using System.Windows;

namespace Revit_AutoExternalWall
{
    /// <summary>
    /// Окно настроек плагина
    /// </summary>
    public partial class SettingsWindow : Window
    {
        public Settings Settings { get; private set; }

        public SettingsWindow(Settings settings)
        {
            InitializeComponent();
            Settings = settings;
            chkCreateOpenings.IsChecked = settings.CreateOpenings;
            chkCopyOpenings.IsChecked = settings.CopyOpeningsToExternalWalls;
        }

        private void BtnOk_Click(object sender, RoutedEventArgs e)
        {
            Settings.CreateOpenings = chkCreateOpenings.IsChecked ?? true;
            Settings.CopyOpeningsToExternalWalls = chkCopyOpenings.IsChecked ?? false;
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

