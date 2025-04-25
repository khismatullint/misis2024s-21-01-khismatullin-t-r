#include "../include/Ransac.h"

namespace RansacNamespace{


/**
 * Конструктор класса для настройки параметров детекции
 */
    settings::settings() {

        video_name = "../data/polos.mp4"; // video_name <-  Путь до видео фрагмента

        sens_for_type = 200; // <- Значение чувствительности к белым пикселям. Чем больше значение, тем больше вероятность обнаружить прерывистую линию.
        fontSize = 1;  //  <- параметры шрифта
        thickness = 2; // <- параметры шрифта
        min_inliers = 20; //  <-  Минимальное количество точек, необходимое для определения линии. Для функции Ransac
        Dist_threshold = 5; // <-  Пороговое значение для определения, является ли точка внутри линии.
        width_line_search = 20; // <- Ширина детектируемой линии
        sense_to_normolize_data = 9; // <- Значение для сглаживания данных, чем больше значение, тем больше потенциальных выбросов

        cout_stripes = 4; // <- Максимальное количество детектируемых линий
        cout_containers = 10; // <- Размер контейнера для учёта предыдущих итераций детекции

        // параметры для milcam

        parametersBird = {381, 350, 557, 350,  0, 531, 906, 533, 608, 371}; // параметры для milcam
        parametersHSV = { 21, 76, 13, 48, 110, 28};

        transformationMatrix << -9.8317563130660944e-03, -3.0483869954955277e-03, 2.7366480699891205e+00,
                                2.9370686204037151e-03, -4.6695660667826495e-02, 1.8811209772606649e+01,
                                1.6456794776922032e-03, -7.1019174142738246e-03, 1.0; // параметры калибровки
    }

/**
 * get_vector_stripes_width - функция для генерации вектора интервалов ширины полос.
 *
 *  width - ширина каждой полосы.
 *  Вектор cv::Point2d, представляющий интервалы ширины полос.
 */
    std::vector<cv::Point2d> settings::get_vector_stripes_width(double width) {
        double min = 0; // Минимальная ширина для первой полосы.
        double max = width; // Максимальная ширина для первой полосы.
        size_t quantity_stripes = cout_stripes; // Количество полос (по какой-то переменной).
        std::vector<cv::Point2d> vec_container_stripes; // Вектор для хранения интервалов ширины полос.

        for (size_t stripes = 1; stripes <= quantity_stripes; stripes++) {
            // Добавляем интервал (минимальная и максимальная ширина) в вектор.
            vec_container_stripes.emplace_back(min, max);

            min += width; // Увеличиваем минимальную ширину для следующей полосы.
            max += width; // Увеличиваем максимальную ширину для следующей полосы.
        }

        return vec_container_stripes; // Возвращаем вектор интервалов ширины полос.
    }
    void settings::save(const std::string& path) const
    {
        cv::FileStorage fs(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
        if(!fs.isOpened()) throw std::runtime_error("cannot open "+path);
    
        fs << "parametersHSV" << parametersHSV
           << "parametersBird" << parametersBird
           << "sens_for_type" << sens_for_type
           << "fontSize" << fontSize
           << "thickness" << thickness
           << "width_line_search" << width_line_search
           << "Dist_threshold" << Dist_threshold
           << "min_inliers" << static_cast<int>(min_inliers)
           << "sense_to_normolize_data" << sense_to_normolize_data
           << "cout_stripes" << static_cast<int>(cout_stripes)
           << "cout_containers" << static_cast<int>(cout_containers);
    }

    settings settings::load(const std::string& path)
    {
        settings s;                       // дефолт
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if(!fs.isOpened()){               // нет файла – оставляем дефолт
            std::cerr<<"settings::load: "<<path<<" not found, using defaults\n";
            return s;
        }
        fs["parametersHSV"] >> s.parametersHSV;
        fs["parametersBird"] >> s.parametersBird;
        fs["sens_for_type"] >> s.sens_for_type;
        fs["fontSize"] >> s.fontSize;
        fs["thickness"] >> s.thickness;
        fs["width_line_search"] >> s.width_line_search;
        fs["Dist_threshold"] >> s.Dist_threshold;
    
        int tmp=0;
        fs["min_inliers"] >> tmp;              s.min_inliers  = static_cast<size_t>(tmp);
        fs["sense_to_normolize_data"] >> s.sense_to_normolize_data;
        fs["cout_stripes"] >> tmp;             s.cout_stripes = static_cast<size_t>(tmp);
        fs["cout_containers"] >> tmp;          s.cout_containers = static_cast<size_t>(tmp);
    
        return s;
    }
}