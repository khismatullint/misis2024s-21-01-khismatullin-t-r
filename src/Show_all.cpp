#include "../include/Ransac.h"


namespace RansacNamespace {

/**
 * Выводит на консоль уравнения линий, хранящихся в контейнере по указанному индексу.
 *
 *  cont      Контейнер, содержащий линии, которые необходимо вывести.
 *  len       Индекс, указывающий на сегмент линий в контейнере, который нужно вывести.
 */
    void cout_line(container &cont, size_t len) {
        std::cout << "\n\nУравнения линий:\n";
        for (mrpt::math::TLine2D line : cont.contain[len]) {
            // Выводим коэффициенты уравнения линии в формате [a, b, c].
            std::cout << "\n [" << line.coefs[0] << ", " << line.coefs[1] << ", " << line.coefs[2] << " ]";
        }
    }

/**
 * Выводит на консоль карту дороги, обозначая обнаруженные линии.
 *
 *  cont      Контейнер, содержащий линии, которые нужно отобразить на карте дороги.
 *  len       Индекс, указывающий на сегмент линий в контейнере, который нужно отобразить.
 *  types      Указатель на вектор с типами линий.
 */
    void show_road_map(container &cont, size_t len, std::vector<bool>& types) {

        std::cout << "\n\nКарта дороги:                             \n..";
        std::string output;
        for (size_t i = 0; i<cont.contain[len].size(); i++) {
            mrpt::math::TLine2D line = cont.contain[len][i];
            bool type = types[i];
            if (line.coefs[2] <= 0) {
                // Если линия не обнаружена (коэффициент c <= 0), выводим символ " ".
                output += "   .";
            } else {
                // Если линия обнаружена (коэффициент c > 0), выводим символ ": | " (в зависимости от типа линии) и увеличиваем счетчик.
                if (type)
                    // сплошная
                    output += " | .";
                else
                    // прерывистая
                    output +=  " : .";
            }

        }
        output += ".\n..";
        for (int i = 0; i< 7; i++)
            std::cout<<output;
    }

/**
 * show_left_right_dist - функция для отображения расстояния до левой и правой полосы на экране.
 *
 *  left_right_distance - вектор, содержащий расстояния (левое и правое) до полосы.
 */
    void show_left_right_dist(std::vector<double>& left_right_distance) {
        std::cout << "\n\nРасстояние до левой и правой полосы:\n";
        std::cout << "\nLeft : " << left_right_distance[0]; // Вывод расстояния до левой полосы.
        std::cout << "\nRight : " << left_right_distance[1]; // Вывод расстояния до правой полосы.
        std::cout << "\n";
    }

}