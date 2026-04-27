#ifndef LITTLEALGCPY_H
#define LITTLEALGCPY_H

#include <vector>
#include <utility>

// Класс подзадачи для алгоритма Литтла
class SubTask {
public:
    float* matrix; //приведённая матрица расстояний
    int size; //размер матрицы
    int unfinished; //сколько осталось незаконченных рёбер
    float estimation; //оценка снизу маршрута в данной подзадаче
    std::vector<int> route; //маршрут, пройденный до данной подзадачи
    std::vector<int> start; //список начал цепочек, в которой находится соответствующий город
    std::vector<int> end; //список концов цепочек, в которой находится соответствующий город
    std::vector<std::pair<int,int>> nulls; //список координат нулей матрицы

    // Конструкторы
    SubTask();
    SubTask(float *matrix, int size);
    SubTask(float *matrix, int unfinished, std::vector<int> route,
            std::vector<int> end, std::vector<int> start, float estimation,
            std::vector<std::pair<int,int>> nulls, int size);

    // Конструктор перемещения
    SubTask(SubTask&& other) noexcept;

    // Оператор перемещения
    SubTask& operator=(SubTask&& other) noexcept;

    // Запрещаем копирование
    SubTask(const SubTask&) = delete;
    SubTask& operator=(const SubTask&) = delete;

    // Деструктор
    ~SubTask();

    // Методы
    float MatrixReduct();
    void GetSubTasks(std::vector<SubTask>& subtasks);
};

// Основная функция алгоритма Литтла
std::vector<int> LittleAlg(float* matrix, int size);

#endif // LITTLEALG_HPP
