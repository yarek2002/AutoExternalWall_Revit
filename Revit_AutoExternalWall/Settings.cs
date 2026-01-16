using System;
using System.IO;
using System.Xml.Serialization;

namespace Revit_AutoExternalWall
{
    /// <summary>
    /// Класс для хранения настроек плагина
    /// </summary>
    [Serializable]
    public class Settings
    {
        /// <summary>
        /// Включено ли создание проемов в внешних стенах
        /// </summary>
        public bool CreateOpenings { get; set; } = true;

        /// <summary>
        /// Включено ли копирование окон и дверей из внутренних стен во внешние стены
        /// </summary>
        public bool CopyOpeningsToExternalWalls { get; set; } = false;

        private static string GetSettingsPath()
        {
            string appDataPath = Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData);
            string settingsDir = Path.Combine(appDataPath, "Revit_AutoExternalWall");
            if (!Directory.Exists(settingsDir))
            {
                Directory.CreateDirectory(settingsDir);
            }
            return Path.Combine(settingsDir, "settings.xml");
        }

        /// <summary>
        /// Загружает настройки из файла
        /// </summary>
        public static Settings Load()
        {
            try
            {
                string settingsPath = GetSettingsPath();
                if (File.Exists(settingsPath))
                {
                    XmlSerializer serializer = new XmlSerializer(typeof(Settings));
                    using (FileStream stream = new FileStream(settingsPath, FileMode.Open))
                    {
                        return (Settings)serializer.Deserialize(stream);
                    }
                }
            }
            catch (Exception)
            {
                // Если не удалось загрузить, возвращаем настройки по умолчанию
            }
            return new Settings();
        }

        /// <summary>
        /// Сохраняет настройки в файл
        /// </summary>
        public void Save()
        {
            try
            {
                string settingsPath = GetSettingsPath();
                XmlSerializer serializer = new XmlSerializer(typeof(Settings));
                using (FileStream stream = new FileStream(settingsPath, FileMode.Create))
                {
                    serializer.Serialize(stream, this);
                }
            }
            catch (Exception)
            {
                // Игнорируем ошибки сохранения
            }
        }
    }
}

