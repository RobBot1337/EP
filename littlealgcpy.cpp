#include "littlealgcpy.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <iterator>

// Вспомогательная функция для отображения матрицы
void ShowMatrix(float* matrix, int size) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            std::cout << matrix[i * size + j] << ' ';
        }
        std::cout << '\n';
    }
}

// Реализация методов класса SubTask

// Конструктор по умолчанию
SubTask::SubTask() : matrix(nullptr), size(0), unfinished(0), estimation(0) {}

// Основной конструктор
SubTask::SubTask(float *matrix, int size) : size(size), unfinished(size), route(size, -1), start(size, -1), end(size, -1) {
    this->matrix = new float[size * size];
    std::copy(matrix, matrix + size * size, this->matrix);

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (matrix[i * size + j] == 0) {
                nulls.push_back(std::make_pair(i, j));
            }
        }
    }
    estimation = this->MatrixReduct();
}

// Полный конструктор
SubTask::SubTask(float *matrix, int unfinished, std::vector<int> route,
                 std::vector<int> end, std::vector<int> start, float estimation,
                 std::vector<std::pair<int,int>> nulls, int size)
    : matrix(matrix), unfinished(unfinished), route(route), start(start),
      end(end), estimation(estimation), nulls(nulls), size(size) {}

// Конструктор перемещения
SubTask::SubTask(SubTask&& other) noexcept
    : matrix(other.matrix), size(other.size), unfinished(other.unfinished),
      estimation(other.estimation), route(std::move(other.route)),
      start(std::move(other.start)), end(std::move(other.end)),
      nulls(std::move(other.nulls)) {
    other.matrix = nullptr;
}

// Оператор перемещения
SubTask& SubTask::operator=(SubTask&& other) noexcept {
    if (this != &other) {
        delete[] matrix;

        matrix = other.matrix;
        size = other.size;
        unfinished = other.unfinished;
        estimation = other.estimation;
        route = std::move(other.route);
        start = std::move(other.start);
        end = std::move(other.end);
        nulls = std::move(other.nulls);

        other.matrix = nullptr;
    }
    return *this;
}

// Деструктор
SubTask::~SubTask() {
    delete[] matrix;
}

// Метод приведения матрицы
float SubTask::MatrixReduct() {
    float reductConst = 0;

    //приведение по строкам
    for (int i = 0; i < size; i++) {
        float min = INFINITY;

        for (int j = 0; j < size; j++) {
            if (matrix[i * size + j] < min) {
                min = matrix[i * size + j];
            }
        }

        reductConst += min;
        for (int j = 0; j < size; j++) {
            if (matrix[i * size + j] == min) {
                auto it = std::find(nulls.begin(), nulls.end(), std::make_pair(i, j));
                if (it == nulls.end() && route[i] == -1) {
                    nulls.push_back(std::make_pair(i, j));
                }
            }
            matrix[i * size + j] -= min;
        }
    }

    //приведение по столбцам
    for (int j = 0; j < size; j++) {
        float min = INFINITY;
        for (int i = 0; i < size; i++) {
            if (matrix[i * size + j] < min) {
                min = matrix[i * size + j];
            }
        }

        reductConst += min;
        for (int i = 0; i < size; i++) {
            if (matrix[i * size + j] == min) {
                auto it = std::find(nulls.begin(), nulls.end(), std::make_pair(i, j));
                if (it == nulls.end() && route[i] == -1) {
                    nulls.push_back(std::make_pair(i, j));
                }
            }
            matrix[i * size + j] -= min;
        }
    }
    return reductConst;
}

// Метод получения подзадач
void SubTask::GetSubTasks(std::vector<SubTask>& subtasks) {
    int nmax = 0;
    int mmax = 0;

    std::vector<std::pair<std::pair<int,int>,std::pair<float,float>>> sortedNulls;
    int k = 0;

    while (k < nulls.size()) {
        float minstr = INFINITY;
        float minrow = INFINITY;

        for (int i = 0; i < size; ++i) {
            if (i != nulls[k].second) {
                if (matrix[nulls[k].first * size + i] < minstr) {
                    minstr = matrix[nulls[k].first * size + i];
                }
            }
        }

        for (int j = 0; j < size; j++) {
            if (j != nulls[k].first) {
                if (matrix[j * size + nulls[k].second] < minrow) {
                    minrow = matrix[j * size + nulls[k].second];
                }
            }
        }

        if (minstr + minrow == INFINITY) {
            int n = nulls[k].first;
            int m = nulls[k].second;

            for (int j = 0; j < size; j++) {
                if (j != m) {
                    if (matrix[size * n + j] == 0) {
                        auto it = std::find(nulls.begin(), nulls.end(), std::make_pair(n, j));
                        if (it != nulls.end()) {
                            if (nulls.begin() + k > it) { k--; }
                            nulls.erase(it);
                        }
                    }
                    matrix[size * n + j] = INFINITY;
                }
            }

            for (int j = 0; j < size; j++) {
                if (j != n) {
                    if (matrix[size * j + m] == 0) {
                        auto it = std::find(nulls.begin(), nulls.end(), std::make_pair(j, m));
                        if (it != nulls.end()) {
                            if (nulls.begin() + k > it) { k--; }
                            nulls.erase(it);
                        }
                    }
                    matrix[size * j + m] = INFINITY;
                }
            }

            if (matrix[size * m + n] == 0) {
                auto it = std::find(nulls.begin(), nulls.end(), std::make_pair(m, n));
                if (it != nulls.end()) {
                    if (nulls.begin() + k > it) { k--; }
                    nulls.erase(it);
                }
            }
            matrix[size * m + n] = INFINITY;
            unfinished--;
            route[n] = m;

            if (start[n] == -1 && start[m] == -1) {
                start[n] = n;
                start[m] = n;
                end[n] = m;
                end[m] = m;
            } else if (start[m] == -1) {
                start[m] = start[n];
                end[m] = m;
                for (int j = 0; j < size; j++) {
                    if (end[j] == n) { end[j] = m; }
                }
            } else if (end[n] == -1) {
                start[n] = n;
                end[n] = end[m];
                for (int j = 0; j < size; j++) {
                    if (start[j] == m) { start[j] = n; }
                }
            } else {
                for (int j = 0; j < size; j++) {
                    if (end[j] == n) { end[j] = end[m]; }
                }
                for (int j = 0; j < size; j++) {
                    if (start[j] == m) { start[j] = start[n]; }
                }
            }
            nulls.erase(nulls.begin() + k);
        } else {
            sortedNulls.push_back(std::make_pair(std::make_pair(nulls[k].first, nulls[k].second),
                                  std::make_pair(minstr, minrow)));
            k++;
        }

        if (unfinished == 2) { return; }
    }

    std::sort(sortedNulls.begin(), sortedNulls.end(),
              [](const std::pair<std::pair<int,int>,std::pair<float,float>>& a,
                 const std::pair<std::pair<int,int>,std::pair<float,float>>& b) {
                  float sumA = a.second.first + a.second.second;
                  float sumB = b.second.first + b.second.second;
                  return sumA > sumB;
              });

    int i = 0;

    while (i < sortedNulls.size()) {
        int from = sortedNulls[i].first.first;
        int to = sortedNulls[i].first.second;

        if (unfinished != 1 && to == 0) {
            i++;
            continue;
        }

        if (unfinished == 1 && to == 0 && std::all_of(start.begin(), start.end(),
                                                      [](int x) { return x == 0; })) {
            break;
        }

        if (end[to] == from && start[from] == to) {
            i++;
            continue;
        }

        break;
    }

    nmax = sortedNulls[i].first.first;
    mmax = sortedNulls[i].first.second;
    int newestimation = estimation + sortedNulls[i].second.first + sortedNulls[i].second.second;

    float* newMatrix1 = new float[size * size];
    std::copy(matrix, matrix + size * size, newMatrix1);

    std::vector<std::pair<int,int>> newnulls;
    std::copy(nulls.begin(), nulls.end(), std::back_inserter(newnulls));
    newnulls.erase(std::find(newnulls.begin(), newnulls.end(), std::make_pair(nmax, mmax)));

    newMatrix1[nmax * size + mmax] = INFINITY;

    for (int j = 0; j < size; j++) {
        if (newMatrix1[j * size + mmax] == sortedNulls[i].second.second &&
            j != nmax &&
            std::find(newnulls.begin(), newnulls.end(), std::make_pair(j, mmax)) == newnulls.end()) {
            newnulls.push_back(std::make_pair(j, mmax));
        }
        newMatrix1[j * size + mmax] -= sortedNulls[i].second.second;
    }

    for (int j = 0; j < size; j++) {
        if (newMatrix1[nmax * size + j] == sortedNulls[i].second.first &&
            j != mmax &&
            std::find(newnulls.begin(), newnulls.end(), std::make_pair(nmax, j)) == newnulls.end()) {
            newnulls.push_back(std::make_pair(nmax, j));
        }
        newMatrix1[nmax * size + j] -= sortedNulls[i].second.first;
    }

    SubTask subtask1 = SubTask(newMatrix1, unfinished, route, end, start, newestimation, newnulls, size);

    auto zero = std::find(nulls.begin(), nulls.end(), std::make_pair(nmax, mmax));
    nulls.erase(zero);

    for (int j = 0; j < size; j++) {
        if (j != mmax) {
            if (matrix[size * nmax + j] == 0) {
                auto it = std::find(nulls.begin(), nulls.end(), std::make_pair(nmax, j));
                if (it != nulls.end()) {
                    nulls.erase(it);
                }
            }
            matrix[size * nmax + j] = INFINITY;
        }
    }

    for (int j = 0; j < size; j++) {
        if (j != nmax) {
            if (matrix[size * j + mmax] == 0) {
                auto it = std::find(nulls.begin(), nulls.end(), std::make_pair(j, mmax));
                if (it != nulls.end()) {
                    nulls.erase(it);
                }
            }
            matrix[size * j + mmax] = INFINITY;
        }
    }

    if (matrix[size * mmax + nmax] == 0) {
        auto it = std::find(nulls.begin(), nulls.end(), std::make_pair(mmax, nmax));
        if (it != nulls.end()) {
            nulls.erase(it);
        }
    }
    matrix[size * mmax + nmax] = INFINITY;

    route[nmax] = mmax;
    estimation = this->MatrixReduct() + estimation;
    unfinished--;

    if (start[nmax] == -1 && start[mmax] == -1) {
        start[nmax] = nmax;
        start[mmax] = nmax;
        end[nmax] = mmax;
        end[mmax] = mmax;
    } else if (start[mmax] == -1) {
        start[mmax] = start[nmax];
        end[mmax] = mmax;
        for (int k = 0; k < size; k++) {
            if (end[k] == nmax) { end[k] = mmax; }
        }
    } else if (end[nmax] == -1) {
        start[nmax] = nmax;
        end[nmax] = end[mmax];
        for (int k = 0; k < size; k++) {
            if (start[k] == mmax) { start[k] = nmax; }
        }
    } else {
        for (int k = 0; k < size; k++) {
            if (end[k] == nmax) { end[k] = end[mmax]; }
        }
        for (int k = 0; k < size; k++) {
            if (start[k] == mmax) { start[k] = start[nmax]; }
        }
    }

    i = 0;
    while (i < nulls.size()) {
        if (nulls[i].first == nmax || nulls[i].second == mmax) {
            nulls.erase(nulls.begin() + i);
        } else {
            i++;
        }
    }

    subtasks.push_back(std::move(subtask1));
}

// Реализация основной функции алгоритма Литтла
std::vector<int> LittleAlg(float* matrix, int size) {
    std::cout << "1. Начало LittleAlg, size=" << size << "\n";
    std::vector<SubTask> subtasks;
    std::cout << "2. Вектор создан, size=" << subtasks.size() << ", capacity=" << subtasks.capacity() << "\n";

    subtasks.reserve(size);
    std::cout << "3. После reserve, capacity=" << subtasks.capacity() << "\n";

    float* cpymatrix = new float[size * size];
    std::copy(matrix, matrix + size * size, cpymatrix);
    std::cout << "4. Матрица скопирована\n";

    SubTask starttask = SubTask(cpymatrix, size);
    std::cout << "5. starttask создан\n";

    subtasks.push_back(std::move(starttask));
    std::cout << "6. starttask перемещен в вектор, размер вектора=" << subtasks.size() << "\n";

    float bestLength = INFINITY;
    std::vector<int> way(size);

    while (!subtasks.empty()) {
        if (subtasks[0].unfinished == 2) {
            std::vector<int> unfinished_cities;
            for (int i = 0; i < size; i++) {
                if (subtasks[0].route[i] == -1) {
                    unfinished_cities.push_back(i);
                }
            }

            int isolated_city = -1;
            for (int i = 0; i < 2; ++i) {
                if (subtasks[0].start[unfinished_cities[i]] == -1) {
                    subtasks[0].route[unfinished_cities[i]] = subtasks[0].start[unfinished_cities[1 - i]];
                    subtasks[0].route[unfinished_cities[1 - i]] = unfinished_cities[i];
                    isolated_city = 1;
                }
            }

            if (isolated_city == -1) {
                subtasks[0].route[unfinished_cities[0]] = subtasks[0].start[unfinished_cities[1]];
                subtasks[0].route[unfinished_cities[1]] = subtasks[0].start[unfinished_cities[0]];
                subtasks[0].estimation += subtasks[0].matrix[size * unfinished_cities[0] +
                                           subtasks[0].start[unfinished_cities[1]]] +
                                         subtasks[0].matrix[size * unfinished_cities[1] +
                                           subtasks[0].start[unfinished_cities[0]]];
            }

            if (subtasks[0].estimation < bestLength) {
                bestLength = subtasks[0].estimation;
                int n = 0;
                for (int i = 0; i < size; i++) {
                    way[i] = subtasks[0].route[n];
                    n = subtasks[0].route[n];
                }
            }

            int i = 0;
            while (i < subtasks.size()) {
                if (subtasks[i].estimation >= bestLength) {
                    subtasks.erase(subtasks.begin() + i);
                } else {
                    i++;
                }
            }
        } else {
            subtasks[0].GetSubTasks(subtasks);
            ShowMatrix(subtasks[0].matrix, size);
            std::cout << '\n';
        }
    }

    delete[] cpymatrix;
    return way;
}
