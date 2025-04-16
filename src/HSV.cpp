#include "../include/Ransac.h"

namespace RansacNamespace {


/**
 * return_hsv - функция для применения цветового фильтра в HSV-цветовом пространстве к изображению.
 *
 *  img - входное изображение, к которому применяется цветовой фильтр.
 *  parameters - вектор параметров цветового фильтра [h1, s1, v1, h2, s2, v2].
 *  Изображение в полутоновом формате, после применения цветового фильтра.
 */
    cv::Mat hsv::return_hsv(const cv::Mat& img, std::vector<int> &parameters) {

        // Извлекаем параметры цветового фильтра из входного вектора.
        int h1 = parameters[0],
                s1 = parameters[1],
                v1 = parameters[2],
                h2 = parameters[3],
                s2 = parameters[4],
                v2 = parameters[5];

        // Создаем объекты Scalar для минимальных и максимальных HSV-значений.
        cv::Scalar h_min = cv::Scalar(h1, s1, v1);
        cv::Scalar h_max = cv::Scalar(h2, s2, v2);

        // Преобразуем входное изображение в HSV-цветовое пространство.
        cv::Mat img_hsv, hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HLS);

        // Применяем цветовой фильтр с учетом минимальных и максимальных значений HSV.
        cv::inRange(hsv, h_min, h_max, img_hsv);
        // Возвращаем полутоновое изображение после применения цветового фильтра.
        return img_hsv;
    }






/**
 * filtered_img - функция для обработки и фильтрации контуров на изображении.
 *
 *  img - входное изображение для обработки.
 *  filtered_contours - вектор, в который будут добавлены отфильтрованные контуры.
 *  filtered_coord - вектор, в который будут добавлены координаты точек отфильтрованных контуров.
 */
    void hsv::filtered_img(const cv::Mat& img, std::vector<std::vector<cv::Point>>& filtered_contours, std::vector<cv::Point>& filtered_coord) {
        // Создание нескольких вспомогательных матриц для обработки изображения.
        cv::Mat thresh, img_result;

        // Копирование входного изображения в матрицу "thresh".
        thresh = img;
        img_result = img;

        // Создание ядра для морфологической операции закрытия (закрытие областей).
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 10));
        cv::erode(thresh, img_result, kernel);

        // Поиск контуров на обработанном изображении.
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img_result, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Фильтрация контуров на основе их размеров и формы.
        for (const auto &cnt: contours) {
            cv::Rect boundingRect = cv::boundingRect(cnt);
            int w = boundingRect.width;
            int h = boundingRect.height;
            double Area = cv::contourArea(cnt, true);
            // Условия для фильтрации контуров.
            if (((h > w * 5) || ((h > 30) && (h < 100))) && ((w < 50) && (w > 3)) && (Area < 700)) {
                filtered_contours.push_back(cnt);
                filtered_coord.insert(filtered_coord.begin(), cnt.begin(), cnt.end());
            }
        }
    }



/**
 * Функция для изменения параметров HSV-фильтрации.
 * Пользователь может настроить значения параметров HSV для фильтрации цветов на изображении.
 * Значения могут быть изменены с использованием ползунков на окне настроек.
 * Результаты настройки могут быть сохранены в файле "hsv_params.txt".
 */
    void hsv::get_parameters(const std::string& video_name) {
        std::cout << "Хотите изменить параметры HSV? (y/n) ";
        char ans;
        int check;
        std::cin >> ans;

        // Инициализация параметров HSV: h1, s1, v1, h2, s2, v2.
        int parameters[6] = {0, 0, 0, 255, 255, 255};
        std::string names[6] = {"h1", "s1", "v1", "h2", "s2", "v2"};
        FILE *f;

        if (ans == 'y') {
            // Блок выполнится, если пользователь хочет изменить параметры.

            cv::Scalar h_max, h_min;
            f = fopen("../data/hsv_params.txt", "r");
            for (int i = 0; i < 6; i++) {
                check = fscanf(f, " %d", parameters + i);
            }

            if (check > 0) {
                fclose(f);
            }

            cv::VideoCapture vid(video_name);
            if (!vid.isOpened()) {
                std::cout << "Ошибка: не удалось открыть камеру." << std::endl;
            }
            cv::namedWindow("result");
            cv::namedWindow("settings");

            // Создание ползунков для настройки параметров.
            for (int i = 0; i < 6; i++) {
                cv::createTrackbar(names[i], "settings", &parameters[i], 255, nullptr);
            }

            while (cv::waitKey(1) != 'w') {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                cv::Mat img;
                vid.read(img);
                cv::Mat hsv;
                cv::cvtColor(img, hsv, cv::COLOR_BGR2HLS);
                cv::waitKey(0);
                // Получение текущих значений параметров HSV.
                int h1 = cv::getTrackbarPos("h1", "settings");
                int s1 = cv::getTrackbarPos("s1", "settings");
                int v1 = cv::getTrackbarPos("v1", "settings");
                int h2 = cv::getTrackbarPos("h2", "settings");
                int s2 = cv::getTrackbarPos("s2", "settings");
                int v2 = cv::getTrackbarPos("v2", "settings");

                // Обновление массива параметров.
                parameters[0] = h1;
                parameters[1] = s1;
                parameters[2] = v1;
                parameters[3] = h2;
                parameters[4] = s2;
                parameters[5] = v2;

                h_min = cv::Scalar(h1, s1, v1);
                h_max = cv::Scalar(h2, s2, v2);

                cv::Mat img_hsv;
                cv::imshow("r", img);
                cv::inRange(hsv, h_min, h_max, img_hsv);
                cv::imshow("result", img_hsv);
                f = fopen("../data/hsv_params.txt", "w");
                for (int parameter : parameters) {
                    fprintf(f, " %d", parameter);
                }
                fclose(f);
            }

            // Сохранение параметров в файле "hsv_params.txt".


            vid.release();
            cv::destroyAllWindows();
        } else {
            // В противном случае параметры фильтрации загружаются из файла.
            f = fopen("../data/hsv_params.txt", "r");
            for (int i = 0; i < 6; i++) {
                check = fscanf(f, " %d", parameters + i);
            }
            if (check > 0) {
                fclose(f);
            }
        }

        // Вывод на экран текущих значений параметров HSV.
        std::cout << "\nВаши параметры HSV: \n\n";
        std::cout << "Минимальные: ";
        std::cout << " h: " << parameters[0];
        std::cout << " s: " << parameters[1];
        std::cout << " v: " << parameters[2];
        std::cout << "\nМаксимальные: ";
        std::cout << " h: " << parameters[3];
        std::cout << " s: " << parameters[4];
        std::cout << " v: " << parameters[5];
        std::cout << "\n\n";
    }



}



