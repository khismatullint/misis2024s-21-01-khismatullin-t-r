#include "../include/Ransac.h"



namespace RansacNamespace {





/**
 * warpImage - функция для выполнения перспективного преобразования изображения.
 *
 * @param img - входное изображение, которое будет преобразовано.
 * @param matrix - вектор матриц преобразования [M, Minv].
 * @param parameters - вектор параметров, включая высоту и ширину выходного изображения.
 * @param type_of_transform - тип преобразования ('n' - из нормального в bird, 'r' - из bird в нормальное).
 * @param img_norm - изначальное зображение для определения разрешения.
 * @return Преобразованное изображение.
 */
    cv::Mat Bird_view::warpImage (const cv::Mat& img, const cv::Mat& img_norm, std::vector<cv::Mat>& matrix, std::vector<int> &parameters, char type_of_transform) {
        cv::Mat warped_img;

        // Получаем матрицу преобразования и обратную матрицу преобразования.
        cv::Mat M = matrix[0];
        cv::Mat Minv = matrix[1];

        int IMAGE_H = parameters[8]; // Высота выходного изображения.
        int IMAGE_W = parameters[9]; // Ширина выходного изображения.

        // Выполняем перспективное преобразование в зависимости от типа преобразования.
        if (type_of_transform == 'n') {
            warped_img = cv::Mat::zeros(IMAGE_H, IMAGE_W, img.type());
            cv::warpPerspective(img, warped_img, M, warped_img.size());
        } else if (type_of_transform == 'r') {

            IMAGE_H = img_norm.rows; // Новая высота для вид сбоку.
            IMAGE_W = img_norm.cols; // Новая ширина для вид сбоку.
            warped_img = cv::Mat::zeros(IMAGE_H, IMAGE_W, img.type());
            cv::warpPerspective(img, warped_img, Minv, warped_img.size());
        }

        return warped_img; // Возвращаем преобразованное изображение.
    }



/**
 * Функция `return_bird_matrix` выполняет перспективную трансформацию изображения
 * и возвращает матрицы преобразования.
 *
 * @param parameters Вектор с параметрами, определяющими координаты точек трансформации
 * и размер выходного изображения.
 * Параметры должны содержать следующие значения:
 *   \n\n
 *   parameters[0] - X-координата точки верхнего левого угла на входном изображении.
 *   \n
 *   parameters[1] - Y-координата точки верхнего левого угла на входном изображении.
 *   \n
 *   parameters[2] - X-координата точки верхнего правого угла на входном изображении.
 *   \n
 *   parameters[3] - Y-координата точки верхнего правого угла на входном изображении.
 *   \n
 *   parameters[4] - X-координата точки нижнего левого угла на входном изображении.
 *   \n
 *   parameters[5] - Y-координата точки нижнего левого угла на входном изображении.
 *   \n
 *   parameters[6] - X-координата точки нижнего правого угла на входном изображении.
 *   \n
 *   parameters[7] - Y-координата точки нижнего правого угла на входном изображении.
 *   \n
 *   parameters[8] - Высота выходного изображения.
 *   \n
 *   parameters[9] - Ширина выходного изображения.
 *
 * @return Возвращает вектор из двух матриц:
 *   - Первая матрица (M) - матрица преобразования из входного изображения в выходное.
 *   - Вторая матрица (Minv) - обратная матрица преобразования из выходного входное изображение.
 */
    std::vector<cv::Mat> Bird_view::return_bird_matrix(std::vector<int> &parameters) {

        int IMAGE_H = parameters[8]; // Высота выходного изображения.
        int IMAGE_W = parameters[9]; // Ширина выходного изображения.

        // Задаем координаты точек входного и выходного изображений для перспективной трансформации.
        cv::Point2f dst[4] = {cv::Point2i(parameters[0], parameters[1]),
                              cv::Point2i(parameters[2], parameters[3]),
                              cv::Point2i(parameters[4], parameters[5]),
                              cv::Point2i(parameters[6], parameters[7])};
        cv::Point2f src[4] = {cv::Point2i(0, 0),
                              cv::Point2i(IMAGE_W, 0),
                              cv::Point2i(0, IMAGE_H),
                              cv::Point2i(IMAGE_W, IMAGE_H)};

        // Получаем матрицу преобразования и обратную матрицу преобразования.
        cv::Mat M = cv::getPerspectiveTransform(dst, src);
        cv::Mat Minv = cv::getPerspectiveTransform(src, dst);

        std::vector<cv::Mat> matrix;
        matrix = {M, Minv};

        return matrix;
    }






/**
 * Метод для установки или изменения параметров для перспективной трансформации.
 * Если пользователь выбирает 'y' (да), то открывается окно с настройками.
 * После настройки параметров, они сохраняются в файл.
 */
    void Bird_view::get_parameters(const std::string& vide_name) {
        cv::VideoCapture vid(vide_name);
        if (!vid.isOpened()) {
            std::cout << "Ошибка: не удалось открыть камеру." << std::endl;
        }
        int check;
        int parameters[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        std::cout << "\nЖелаете изменить параметры обзора сверху? (y/n) ";
        char ans;
        std::cin >> ans;
        if (ans == 'y') {

            FILE *f;
            f = fopen("../data/bird_params.txt", "r");
            for (int i = 0; i < 10; i++) {
                check = fscanf(f, " %d", parameters + i);
            }
            if (check > 0) {
                fclose(f);
            }

            cv::namedWindow("result");
            cv::namedWindow("start video");
            cv::namedWindow("settings");

            cv::Mat img;
            vid.read(img);
            int height = img.rows;
            int width = img.cols;

            parameters[8] = height;
            parameters[9] = width;

            // Создаем ползунки для настройки параметров.
            cv::createTrackbar("1 point h", "settings", &parameters[1], height, nullptr);
            cv::createTrackbar("1 point w", "settings", &parameters[0], width, nullptr);
            cv::createTrackbar("2 point h", "settings", &parameters[3], height, nullptr);
            cv::createTrackbar("2 point w", "settings", &parameters[2], width, nullptr);
            cv::createTrackbar("3 point h", "settings", &parameters[5], height, nullptr);
            cv::createTrackbar("3 point w", "settings", &parameters[4], width, nullptr);
            cv::createTrackbar("4 point h", "settings", &parameters[7], height, nullptr);
            cv::createTrackbar("4 point w", "settings", &parameters[6], width, nullptr);
            cv::createTrackbar("image h", "settings", &parameters[8], height, nullptr);
            cv::createTrackbar("image w", "settings", &parameters[9], width, nullptr);

            while (cv::waitKey(1) != 'e') {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                vid.read(img);
                cv::waitKey(0);
                int p1h = cv::getTrackbarPos("1 point h", "settings");
                int p1w = cv::getTrackbarPos("1 point w", "settings");
                int p2h = cv::getTrackbarPos("2 point h", "settings");
                int p2w = cv::getTrackbarPos("2 point w", "settings");
                int p3h = cv::getTrackbarPos("3 point h", "settings");
                int p3w = cv::getTrackbarPos("3 point w", "settings");
                int p4h = cv::getTrackbarPos("4 point h", "settings");
                int p4w = cv::getTrackbarPos("4 point w", "settings");
                int h = cv::getTrackbarPos("image h", "settings");
                int w = cv::getTrackbarPos("image w", "settings");

                // Обновляем параметры.
                parameters[0] = p1w;
                parameters[1] = p1h;
                parameters[2] = p2w;
                parameters[3] = p2h;
                parameters[4] = p3w;
                parameters[5] = p3h;
                parameters[6] = p4w;
                parameters[7] = p4h;
                parameters[8] = h;
                parameters[9] = w;

                cv::Mat bird_image;
                int IMAGE_H = parameters[8];
                int IMAGE_W = parameters[9];
                cv::Point2f dst[4] = {cv::Point2i(parameters[0], parameters[1]),
                                      cv::Point2i(parameters[2], parameters[3]),
                                      cv::Point2i(parameters[4], parameters[5]),
                                      cv::Point2i(parameters[6], parameters[7])};
                cv::Point2f src[4] = {cv::Point2i(0, 0),
                                      cv::Point2i(IMAGE_W, 0),
                                      cv::Point2i(0, IMAGE_H),
                                      cv::Point2i(IMAGE_W, IMAGE_H)};
                cv::Mat M = cv::getPerspectiveTransform(dst, src);
                cv::Mat Minv = cv::getPerspectiveTransform(src, dst);

                cv::Mat warped_img;
                warped_img = cv::Mat::zeros(IMAGE_H, IMAGE_W, img.type());
                cv::warpPerspective(img, warped_img, M, warped_img.size());
                bird_image = warped_img;

                cv::imshow("result", bird_image);

                // Рисуем точки на изображении и выводим параметры.
                cv::circle(img, cv::Point(p1w, p1h), 3, cv::Scalar(0, 0, 255), -1);
                cv::circle(img, cv::Point(p2w, p2h), 3, cv::Scalar(0, 0, 255), -1);
                cv::circle(img, cv::Point(p3w, p3h), 3, cv::Scalar(0, 0, 255), -1);
                cv::circle(img, cv::Point(p4w, p4h), 3, cv::Scalar(0, 0, 255), -1);

                cv::putText(img, "P 1 : (" + std::to_string(p1w) + ", " + std::to_string(p1h) + ")",
                            cv::Point(p1w + 10, p1h - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                cv::putText(img, "P 2 : (" + std::to_string(p2w) + ", " + std::to_string(p2h) + ")",
                            cv::Point(p2w + 10, p2h - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                cv::putText(img, "P 3 : (" + std::to_string(p3w) + ", " + std::to_string(p3h) + ")",
                            cv::Point(p3w + 10, p3h - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                cv::putText(img, "P 4 : (" + std::to_string(p4w) + ", " + std::to_string(p4h) + ")",
                            cv::Point(p4w + 10, p4h - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

                cv::imshow("start video", img);
                f = fopen("../data/bird_params.txt", "w");
                for (int parameter : parameters) {
                    fprintf(f, " %d", parameter);
                }
            }

            // Сохраняем параметры в файл.

            fclose(f);

            cv::destroyAllWindows();
        } else {
            // Если пользователь выбирает 'n' (нет), то параметры загружаются из файла.
            FILE *f = fopen("../data/bird_params.txt", "r");
            for (int i = 0; i < 10; i++) {
                check = fscanf(f, " %d", parameters + i);
            }
            if (check > 0) {
                fclose(f);
            }
        }

        // Вывод параметров.
        std::cout << "\nПараметры обзора сверху: \n";
        std::cout << "\nP 1 : x " << parameters[0] << " y " << parameters[1] << "\n";
        std::cout << "P 2 : x " << parameters[2] << " y " << parameters[3] << "\n";
        std::cout << "P 3 : x " << parameters[4] << " y " << parameters[5] << "\n";
        std::cout << "P 4 : x " << parameters[6] << " y " << parameters[7] << "\n\n";
    }


}
