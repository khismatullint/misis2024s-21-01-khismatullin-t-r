#include "../include/Ransac.h"

namespace RansacNamespace{

/**
 * return_type_of_line - функция для определения типа линий на основе количества контуров в них.
 *
 * @param count_contours_in_line - вектор, содержащий количество контуров в каждой линии.
 * @param result_type_of_lines - вектор, в который будет записан результат определения типа линий.
 *                              Если линия содержит более одного контура, тип будет "false", иначе "true".
 */
    void return_type_of_line(std::vector<int>& count_contours_in_line, std::vector<bool>& result_type_of_lines) {
        result_type_of_lines.clear(); // Очищаем вектор результатов.

        for (size_t i = 0; i < count_contours_in_line.size(); i++) {
            if (count_contours_in_line[i] > 1) {
                result_type_of_lines.push_back(false); // Линия содержит более одного контура, тип "false".
            } else {
                result_type_of_lines.push_back(true); // Линия содержит один контур, тип "true".
            }
        }
    }


}