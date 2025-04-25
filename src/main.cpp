#include "../include/Ransac.h"
#include "../include/Dataset.h"
#include "../include/AutoTune.h"
#include "../include/Ransac.h"

int main(int argc, char** argv) {

    RansacNamespace::settings init;
    bool doTune = (argc>1 && std::string(argv[1]) == "--tune");
    
    if(doTune){
        RansacNamespace::Dataset train("../dataset/culane/train");
        init = RansacNamespace::autoTune(init, train, 1500);
        init.save("../data/auto_params.yml");
        std::cout<<"Лучшие параметры сохранены"<<std::endl;
    }else{
        init = RansacNamespace::settings::load("../data/auto_params.yml");
    }
    // Создание объекта для обработки изображения птичьего вида и настройки его параметров.
    RansacNamespace::Bird_view bird_img;
    bird_img.get_parameters(init.video_name);

    // Создание объекта для обработки изображения HSV и настройки его параметров.
    RansacNamespace::hsv hsv_img;
    RansacNamespace::hsv::get_parameters(init.video_name);
    // Открытие видеопотока.
    cv::VideoCapture vid(init.video_name);
    if (!vid.isOpened()) {
        std::cout<< "Ошибка: не удалось открыть камеру." << std::endl;
    }

    cv::namedWindow("fif1");
    cv::Mat img;
    cv::Mat hsv;
    cv::Mat bird;
    RansacNamespace::TL lines;
    std::vector<int> I1(init.cout_stripes);
    std::vector<int> I2(init.cout_stripes);
    std::vector<bool> buffBoolList(init.cout_stripes);
    std::vector<std::vector<cv::Point>> polygon(0);
    polygon = {{{}}};
    std::vector<bool> result_type_of_lines(init.cout_stripes);
    // Инициализация массивов и параметров.
    for (size_t i = 0; i < init.cout_stripes; i++) {
        I1[i] = 0;
    }
    for (size_t i = 0; i < init.cout_stripes; i++) {
        I2[i] = 999;
    }
    for (size_t i = 0; i < init.cout_stripes; i++) {
        buffBoolList[i] = true;
    }

    cv::Scalar textColor = {0, 0, 0};
    cv::Point textPosition = {100, 100};
    size_t iteration = 0;

    std::vector<double> left_right_distance;

    // Создание контейнера для хранения результатов обработки.
    RansacNamespace::container cont (init.cout_containers, init.cout_stripes, static_cast<size_t>( init.parametersBird[9]));
    RansacNamespace::container cont_poly (init.cout_containers, init.cout_stripes, static_cast<size_t>( init.parametersBird[9]));

    std::vector<cv::Mat> matrixBird;
    std::cout << std::endl << "Запуск обнаружения линий..." << std::endl << std::endl;

    while (cv::waitKey(1) != 'q') {
        mrpt::system::CTicTac tictac; // Таймер для измерения времени выполнения.

        if (iteration < 20) {
            iteration++;
        } // общий итератор цикла
        vid.read(img);

        // Получение вектора определяющего ширину полос, на которые потом будет делиться изображение
        std::vector<cv::Point2d> vec_container_stripes = init.get_vector_stripes_width(cont.width_stripes);

        if (iteration == 1) {
            // Получние матрицы преобразования в Птичью перспективу
            matrixBird = bird_img.return_bird_matrix(init.parametersBird);
        }


        bird = bird_img.warpImage(img, img, matrixBird, init.parametersBird, 'n'); // Приенение матрицы
        hsv = hsv_img.return_hsv(bird, init.parametersHSV); // получение полутонового изображения
        std::vector<std::vector<cv::Point>> contours = {};
        std::vector<cv::Point> coord = {};
        //фильтрация полученных контуров
        hsv_img.filtered_img(hsv, contours, coord);

        cv::Mat test_img = cv::Mat::zeros(hsv.rows, hsv.cols, CV_8UC3);
        cv::drawContours(test_img, contours, -1, cv::Scalar::all(255), -1);

//        RansacNamespace::draw_inliers(bird, coord);

        // Применение RANSAC для обнаружения линий.
        lines = RansacNamespace::RANSACLines(coord, init.min_inliers, init.Dist_threshold);

        // Удаление наклонных линий.
        RansacNamespace::rm_slanted_lines(lines);
        //Разделить изображение на полосы и выделить в каждой из них свою линию разметки
        RansacNamespace::division_into_stripes(lines, cont, vec_container_stripes);

        // Поиск координат для нахождения полиномов.
        std::vector<std::vector<cv::Point>> coord_for_lines;
        RansacNamespace::find_x_y(lines, contours, init.width_line_search, coord_for_lines, result_type_of_lines);
        //Расчёт полиномов из полученных ранее координат
        RansacNamespace::TL Polylines = RansacNamespace::x_y_to_polynom(coord_for_lines);


        // Добавление результатов в контейнер и нормализация данных.
        cont_poly.add_to_container(Polylines, cont_poly.contain);
        if (iteration > 10)
            cont_poly.normalizeData(cont_poly.contain, I1, buffBoolList, I2, init.sense_to_normolize_data);

        // Вывод параметров полинома и отображение карты дороги.
        RansacNamespace::cout_line(cont_poly, init.cout_containers-1);
        std::cout << "\n";

        RansacNamespace::show_road_map(cont_poly, init.cout_containers-1, result_type_of_lines);


        // Отображение линий и пространства дороги.
        cv::Mat line_image = cv::Mat::zeros(bird.size(), bird.type());
        RansacNamespace::draw_lines(line_image, Polylines, true, result_type_of_lines);
//        RansacNamespace::draw_lines(bird, lines, false, result_type_of_lines);

        //Получение дистанции до левой и правой полосы


        std::vector<std::vector<cv::Point2d>> Three_points = RansacNamespace:: get_three_point_vector(Polylines, line_image, init.transformationMatrix, left_right_distance);

        bool lines_detected = RansacNamespace::lines_found(Polylines);
        std::cout<< "; Значение lines_detected: "<< lines_detected;

        //Применение матрицы преобразования в обратном режиме
        cv::imshow("fif2", bird);
        line_image = bird_img.warpImage(line_image, img, matrixBird, init.parametersBird , 'r');
        line_image = line_image + img;

        // Отображение информации о расстоянии.
        std::stringstream ss1;
        textPosition = {400, 100};
        ss1 << std::fixed << std::setprecision(2) << left_right_distance[0];
        std::string numberString1 = ss1.str();
        putText(line_image,  "left "+numberString1+" m", textPosition, cv::FONT_HERSHEY_SIMPLEX, init.fontSize, textColor, init.thickness);

        std::stringstream ss2;
        textPosition.y += 50;
        ss2 << std::fixed << std::setprecision(2) << left_right_distance[1];
        std::string numberString2 = ss2.str();
        putText(line_image, "right "+numberString2+" m", textPosition, cv::FONT_HERSHEY_SIMPLEX, init.fontSize, textColor, init.thickness);
        cv::imshow("fif1", line_image);

        RansacNamespace::show_left_right_dist(left_right_distance);

        std::cout << "\n Время вычислений для одного кадра: " << tictac.Tac()  << " с" << std::endl;
        std::cout << std::endl << std::endl << "/" << std::string(100,'-') << "/" << std::endl;
        cv::waitKey(0);
    }

    return 0;
}