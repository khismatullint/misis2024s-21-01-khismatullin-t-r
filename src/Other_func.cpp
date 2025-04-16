#include "../include/Ransac.h"

namespace RansacNamespace {


/**
 * Функция `division_into_stripes` выполняет разделение набора линий на группы,
 * относящиеся к различным полосам (или полоскам).
 *
 *  lines Вектор линий, которые необходимо разделить на полосы.
 *  cont Контейнер, в который будут добавлены линии, отнесенные к полосам.
 *  vector_stripes_widh Вектор, содержащий диапазоны ширин полос,
 * где каждый элемент представляет собой интервал [min, max], где min - минимальная ширина,
 * а max - максимальная ширина полосы.
 */
    void division_into_stripes(TL &lines, container &cont, std::vector<cv::Point2d>& vector_stripes_widh) {
        TL good_lines; // Создаем вектор для хранения линий, относящихся к полосам.
        good_lines.clear();
        good_lines.resize(cont.quantity_stripes);
        double min, max;

        // Проходим по каждой полосе и выбираем линии, которые находятся в данной полосе.
        for (size_t i = 0; i < vector_stripes_widh.size(); i++) {
            min = vector_stripes_widh[i].x;
            max = vector_stripes_widh[i].y;
            for (auto& line : lines) {
                double x_sample = -line.coefs[2] / line.coefs[1]; // Вычисляем x-координату пересечения с осью Y.
                if ((x_sample > min) && (x_sample < max)) {
                    good_lines[i] = line;
                }
            }
        }

        // Добавляем линии в контейнер и обновляем вектор линий.
        cont.add_to_container(good_lines, cont.contain);
        lines.clear();
        lines = good_lines;
    }


/**
 * Удаляет наклонные линии из заданного вектора линий.
 *
 *  lines     Вектор линий, из которого нужно удалить наклонные линии.
 *           Вектор линий без наклонных линий.
 */
    void rm_slanted_lines(TL &lines) {
        TL good_lines; // Создаем вектор для хранения "хороших" (не наклонных) линий.
        good_lines.resize(lines.size());

        // Проходим по каждой линии и проверяем угол наклона, удаляем линии с большим углом наклона.
        for (size_t i = 0; i < lines.size(); i++) {
            double tangle = -(lines[i].coefs[1]) / lines[i].coefs[0]; // Вычисляем угол наклона.
            if (abs(tangle) > 7) { // Пороговое значение для угла наклона.
                good_lines[i] = lines[i]; // Добавляем "хорошую" линию в результат.
            }
        }
        lines.clear();
        lines = good_lines;
    }


/**
 * Проверяет наличие обнаруженных линий в векторе линий.
 *
 *  lines Вектор прямых линий для анализа.
 *  bool Возвращает true, если обнаружены линии, иначе - false.
 */
    bool lines_found(TL& lines) {
        bool no_lines_detected = true; // Переменная для отслеживания отсутствия обнаруженных линий
        size_t count_of_line = 0; // Счетчик обнаруженных линий

        // Перебор каждой линии в векторе lines
        for (auto l : lines) {
            // Проверка коэффициента l.coefs[2] на положительное значение
            if ((l.coefs[2] > 0) || (l.coefs[2] < 0))
                count_of_line++; // Увеличение счетчика обнаруженных линий
        }

        // Проверка количества обнаруженных линий
        if (count_of_line == 0) {
            no_lines_detected = false; // Обнаружены линии
            std::cout << "Линий не обнаружено";
        } else {
            std::cout << "Линии обнаружены";
        }

        return no_lines_detected; // Возвращается результат: true - если линии не обнаружены, иначе - false
    }

}


