
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include <cmath>
#include <thread>
#include <Eigen/Dense>

#include <mrpt/math/ransac_applications.h>
#include <mrpt/system/CTicTac.h>



namespace RansacNamespace {

    typedef std::vector<std::vector<double>> vec;
    typedef  mrpt::math::CVectorDouble CVD ;
    typedef std::vector<mrpt::math::TLine2D> TL;

    /// settings.cpp
    class settings{
    public:
        /// Путь до видео фрагмента
        std::string video_name;
        ///  Значение чувствительности к белым пикселям. Чем больше значение, тем больше вероятность обнаружить прерывистую линию.
        int sens_for_type;
        ///  Параметры шрифта
        int fontSize;
        /// Параметры шрифта
        int thickness;
        ///  Ширина детектируемой линии
        int width_line_search;
        ///   Пороговое значение для определения, является ли точка внутри линии.
        double Dist_threshold;
        ///  Значение для сглаживания данных, чем больше значение, тем больше потенциальных выбросов
        int sense_to_normolize_data;

        ///  Максимальное количество детектируемых линий
        size_t cout_stripes;
        /// Минимальное количество точек, необходимое для определения линии. Для функции Ransac
        size_t min_inliers;
        ///  Размер контейнера для учёта предыдущих итераций детекции
        size_t cout_containers;

        /// Параметры для bird преобразования (подробнее в return_bird_matrix())
        std::vector<int> parametersBird;
        /// Вектор параметров цветового фильтра [h1, s1, v1, h2, s2, v2]
        std::vector<int> parametersHSV;

        double w1920_to_width;
        double h1080_to_height;

        ///Параметры калибровки
        Eigen::Matrix3d transformationMatrix;
        settings();
        std::vector<cv::Point2d> get_vector_stripes_width(double width);
    };

    ///Bird_view.cpp
    class Bird_view {
    public:
        static cv::Mat warpImage (const cv::Mat& img, const cv::Mat& img_norm, std::vector<cv::Mat>& matrix,std::vector<int> &parameters, char type_of_transform = 'n');
        void get_parameters(const std::string& vide_name);
        static std::vector<cv::Mat> return_bird_matrix(std::vector<int> &parameters);
    };


    ///HSV.cpp
    class hsv {
    public:
        static cv::Mat return_hsv(const cv::Mat& img, std::vector<int> &parameters);
        static void get_parameters(const std::string& vide_name);
        static  void filtered_img(const cv::Mat& img,  std::vector<std::vector<cv::Point>>& filtered_contours, std::vector<cv::Point>& filtered_coord);
    };


    ///dashed_lines.cpp
    void return_type_of_line(std::vector<int>& count_contours_in_line, std::vector<bool>& result_type_of_lines);

    ///container.cpp
    class container {

    private:
        static void check_boolean_list(std::vector<bool> &booleanList, std::vector<bool> &buffBoolList, std::vector<int> &I, int sense, size_t i);
        static void delete_space(std::vector<TL> &container,
                                 std::vector<int> &I, std::vector<int> &I2,
                                 std::vector<bool> &booleanList, std::vector<bool> &buffBoolList,
                                 size_t len, int sense );


    public:
        std::vector<TL> contain;
        size_t quantity_container;
        size_t quantity_stripes;
        double width_stripes;

        container(size_t s, size_t l, size_t img_width);

        static void add_to_container(TL lines, std::vector<TL> &cont);
        static void normalizeData(std::vector<TL> &container,
                                  std::vector<int> &I, std::vector<bool> &buffBoolList, std::vector<int> &I2,
                                  int sense);
    };
    /// draw.cpp
    vec param_to_coord(mrpt::math::TLine2D line, size_t rows, bool Polynom);
    void draw_lines(cv::Mat image, TL &lines, bool Polynom, std::vector<bool>& result_type_of_lines);
    void draw_inliers(cv::Mat image, std::vector<cv::Point>& coord);

    ///Other_func.cpp
    void division_into_stripes(TL &lines, container &cont,std::vector<cv::Point2d>& vector_stripes_widh);
    void rm_slanted_lines(TL &lines);
    bool lines_found (TL& lines);

    ///show_all.cpp
    void cout_line(container &cont, size_t len);
    void show_road_map(container &cont, size_t len, std::vector<bool>& result_type_of_lines);
    void show_left_right_dist (std::vector<double>& left_right_distance);

    /// Ransac.cpp
    TL RANSACLines(std::vector<cv::Point>& coords, size_t min_inliers, double DIST_THRESHOLD);

    /// simple_line_to_polynom.cpp
    TL x_y_to_polynom(std::vector<std::vector<cv::Point>>& coord_for_lines);
    void find_x_y(TL &lines, std::vector<std::vector<cv::Point>>& contours , double width,
                  std::vector<std::vector<cv::Point>>& result_coord, std::vector<bool>& result_type_of_lines);

    ///distance_to_lane.cpp
    cv::Point return_xy_low_point(mrpt::math::TLine2D &line, int y);
    std::vector<cv::Point> three_dots_l_r(int x1, int x2, int x3, mrpt::math::TLine2D &line);
    void return_three_vec_point_in_img_coord (std::vector<std::vector<cv::Point>>& points);
    cv::Point2d find_distance_point_to_center(cv::Point2d Center, cv::Point point);
    std::vector<std::vector<cv::Point2d>>  get_three_point_vector(TL lines, const cv::Mat &image, Eigen::Matrix3d& transformationMatrix, std::vector<double>& left_right_distance);
}