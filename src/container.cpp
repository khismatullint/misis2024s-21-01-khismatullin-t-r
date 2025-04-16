#include "../include/Ransac.h"


namespace RansacNamespace {



/**
* Конструктор класса `container`.
*
* @param s             Количество контейнеров для хранения линий.
* @param l             Количество сегментов (полос) для хранения линий в каждом контейнере.
* @param img_width     Ширина изображения.
*/
    container::container(size_t s, size_t l, size_t img_width) {
        quantity_container = s; // Устанавливаем количество контейнеров.
        quantity_stripes = l; // Устанавливаем количество полос.
        width_stripes = static_cast<double>(img_width) / static_cast<double>(l); // Рассчитываем ширину каждой полосы.
        contain.resize(quantity_container); // Изменяем размер контейнера для хранения линий.
    }

/**
 * Добавляет вектор линий в контейнер и удаляет старые данные.
 *
 * @param lines         Вектор линий для добавления в контейнер.
 * @param cont          Вектор контейнеров для хранения линий.
 */
    void container::add_to_container(TL lines, std::vector<TL> &cont) {
        cont.erase(cont.begin()); // Удаляем старый контейнер.
        cont.push_back(lines); // Добавляем новый вектор линий в контейнер.
    }

/**
 * Функция `check_boolean_list` проверяет и обновляет булевый список на основе изменений и счетчика.
 *
 * @param booleanList Вектор булевых значений, который требуется проверить и обновить.
 * @param buffBoolList Буферный вектор булевых значений для сравнения с основным.
 * @param I Вектор счетчиков изменений для каждого элемента в булевом списке.
 * @param sense Порог изменений, после которого булевое значение считается измененным.
 * @param i Индекс элемента, который нужно проверить и обновить в списках.
 */
    void container::check_boolean_list(std::vector<bool> &booleanList, std::vector<bool> &buffBoolList, std::vector<int> &I, int sense, size_t i) {
        if (buffBoolList[i] != booleanList[i]) { // Проверяем, изменилось ли текущее булевое значение.
            if (I[i] <= sense) {
                booleanList[i] = buffBoolList[i]; // Обновляем булевое значение.
                I[i]++; // Увеличиваем счетчик изменений.
            } else {
                I[i] = 0; // Сбрасываем счетчик изменений.
                buffBoolList[i] = booleanList[i]; // Обновляем буферное булевое значение.
            }
        } else {
            I[i] = 0; // Сбрасываем счетчик изменений, если булевое значение не изменилось.
        }
    }

/**
 * Функция `delete_space` осуществляет удаление "пустых" линий в контейнере,
 * где "пустые" линии имеют нулевые коэффициенты (не значимые) и обновляет
 * контейнер с учетом порога изменений и счетчиков изменений.
 *
 * @param container Вектор линий, в котором выполняется удаление "пустых" линий.
 * @param I1 Вектор счетчиков изменений для каждой линии в контейнере.
 * @param I2 Вектор счетчиков для "пустых" линий.
 * @param booleanList Вектор булевых значений, определяющих изменения в линиях.
 * @param buffBoolList Буферный вектор булевых значений для сравнения с основным.
 * @param len Индекс текущего контейнера (списка линий).
 * @param sense Порог изменений, после которого булевое значение считается измененным.
 */
    void container::delete_space(std::vector<TL> &container,
                                 std::vector<int> &I1, std::vector<int> &I2,
                                 std::vector<bool> &booleanList, std::vector<bool> &buffBoolList,
                                 size_t len, int sense) {
        for (size_t i = 0; i < container[len].size(); i++) {
            // Проверяем, является ли текущая линия "пустой" (без значимых коэффициентов).
            if (int(container[len][i].coefs[2]) == 0) {
                I2[i]++;
                if (I2[i] < sense) {
                    size_t k = 1;
                    container[len][i] = container[len - k][i]; // Заменяем текущую линию на предыдущую.
                } else {
                    I2[i] = 0; // Сбрасываем счетчик изменений для "пустой" линии.
                }
            }
            check_boolean_list(booleanList, buffBoolList, I1, sense, i); // Проверяем и обновляем булевый список.

            if (int(container[len][i].coefs[2]) == 0) {
                if (booleanList[i]) {
                    size_t k = 1;
                    container[len][i] = container[len - k][i]; // Заменяем текущую линию на предыдущую.
                } else {
                    container[len][i] = {0, 0, 0}; // Если булевое значение ложное, устанавливаем линию как пустую.
                }
            }
        }
    }


/**
 * Нормализует данные в контейнере линий на основе заданных условий.
 *
 * @param container     Вектор контейнеров для хранения линий.
 * @param I1            Вектор счетчиков для каждой линии в списке.
 * @param buffBoolList  Буферный список булевых значений.
 * @param I2            Вектор счетчиков для каждой линии в списке.
 * @param sense         Пороговое значение для нормализации данных.
 */
    void container::normalizeData(std::vector<TL> &container,
                                  std::vector<int> &I1, std::vector<bool> &buffBoolList, std::vector<int> &I2,
                                  int sense) {
        std::vector<bool> booleanList; // Создаем вектор булевых значений для каждой линии.
        size_t len = container.size() - 1; // Получаем индекс текущего контейнера линий.

        // Заполняем вектор booleanList значениями, указывающими на наличие линий в текущем контейнере.
        for (mrpt::math::TLine2D line : container[len]) {
            booleanList.push_back(static_cast<int>(line.coefs[2]) != 0);
        }

        // Вызываем функцию delete_space для удаления лишних данных в текущем контейнере линий.
        delete_space(container, I1, I2, booleanList, buffBoolList, len, sense);
    }


}

