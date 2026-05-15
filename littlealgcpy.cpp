#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <utility>

void ShowMatrix(float* matrix,int size){
    for (int i=0;i<size;i++){
        for (int j=0;j<size;j++){
            std::cout<<matrix[i*size+j]<<' ';
        }
        std::cout<<'\n';
    }
}
class SubTask{
public:
    float* matrix; //приведённая матрица расстояний
    int size; //размер матрицы
    int unfinished; //сколько осталось незаконченных рёбер
    float estimation; //оценка снизу маршрута в данной подзадаче
    std::vector<int> route; //маршрут, пройденный до данной подзадачи
    std::vector<int> start; //список начал цепочек, в которой находится соответствующий город
    std::vector<int> end; //список концов цепочек, в которой находится соответствующий город
    std::vector<std::pair<int,int>> nulls; //список координат нулей матрицы
    SubTask() : matrix(nullptr), size(0), unfinished(0), estimation(0) {
        // Можно оставить пустым или добавить минимальную инициализацию
    }

    SubTask(float *matrix,int unfinished,std::vector<int> route, std::vector<int> end,std::vector<int> start,float estimation,std::vector<std::pair<int,int>> nulls,int size): matrix(matrix),
                                                                                        unfinished(unfinished),
                                                                                        route(route),
                                                                                        start(start),
                                                                                        end(end),
                                                                                        estimation(estimation),
                                                                                        nulls(nulls),
                                                                                        size(size){}
    // Конструктор перемещения
    SubTask(SubTask&& other) noexcept
        : matrix(other.matrix),  // забираем указатель
        size(other.size),
        unfinished(other.unfinished),
        estimation(other.estimation),
        route(std::move(other.route)),
        start(std::move(other.start)),
        end(std::move(other.end)),
        nulls(std::move(other.nulls)) {

        other.matrix = nullptr;  // чтобы other не удалил данные
    }

    // Оператор перемещения
    SubTask& operator=(SubTask&& other) noexcept {
        if (this != &other) {
            delete[] matrix;  // освобождаем свои данные

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

    // Запрещаем копирование
    SubTask(const SubTask&) = delete;
    SubTask& operator=(const SubTask&) = delete;
    float MatrixReduct(){

        float reductConst=0;
        //приведение по строкам
        for (int i=0;i<(size);i++){
            float min=INFINITY;

            for (int j=0;j<size;j++){
                if (matrix[i*size+j]<min){
                    min=matrix[i*size+j];
                }
            }

            reductConst+=min;
            for (int j=0;j<size;j++){
                if (matrix[i*size+j]==min){
                    auto it=std::find(nulls.begin(),nulls.end(),std::make_pair(i,j));
                    if (it==nulls.end() && route[i]==-1){
                        nulls.push_back(std::make_pair(i,j));
                    }
                }
                matrix[i*size+j]-=min;
            }
        }
        //приведение по столбцам
        for (int j=0;j<size;j++){
            float min=INFINITY;
            for (int i=0;i<size;i++){
                if (matrix[i*size+j]<min){
                    min=matrix[i*size+j];

                }
            }

            reductConst+=min;
            for (int i=0;i<size;i++){
                if (matrix[i*size+j]==min){
                    auto it=std::find(nulls.begin(),nulls.end(),std::make_pair(i,j));
                    if (it==nulls.end() && route[i]==-1){
                        nulls.push_back(std::make_pair(i,j));
                    }
                }
                matrix[i*size+j]-=min;
            }
        }
        return reductConst;
    }
    SubTask(float *matrix,int size):size(size),unfinished(size),route(size,-1),start(size,-1),end(size,-1){
        this->matrix = new float[size * size];
        std::copy(matrix, matrix + size * size, this->matrix);


        for (int i=0;i<size;i++){// находим исходные нули
            for (int j=0;j<size;j++){
                if (matrix[i*size+j]==0){
                    nulls.push_back(std::make_pair(i,j));
                }
            }
        }
        estimation=this->MatrixReduct();//начальная оценка
    }
    ~SubTask(){
        delete[] matrix;
    }

    void GetSubTasks(std::vector<SubTask>& subtasks){
        int nmax=0;
        int mmax=0;
        float maxReduct=0;
        int kmax=0;
        float Maxminstr=INFINITY;

        float Maxminrow=INFINITY;
        std::vector<std::pair<std::pair<int,int>,std::pair<float,float>>> sortedNulls;
        int k=0;
        while (k<nulls.size()){
            float minstr=INFINITY;

            float minrow=INFINITY;

            float reductAdd=0;
            for (int i=0;i<size;++i){
                if (i!=nulls[k].second){
                    if (matrix[nulls[k].first*size+i]<minstr){ //поиск минимального в строке
                        minstr=matrix[nulls[k].first*size+i];
                        if (minstr==0){break;}
                    }
                }
            }
            for (int j=0;j<size;j++){
                if (j!=nulls[k].first){
                    if (matrix[j*size+nulls[k].second]<minrow){//поиск минимального в столбце
                        minrow=matrix[j*size+nulls[k].second];
                        if (minrow==0){break;}

                    }
                }
            }
            if (minstr+minrow==INFINITY){

                int n=nulls[k].first;
                int m=nulls[k].second;
                for (int j=0;j<size;j++){//если оценка бесконечность, то это фиксированное ребро
                    if (j!=m){
                        if (matrix[size*n+j]==0){
                            auto it=std::find(nulls.begin(),nulls.end(),std::make_pair(n,j)); //если в строке с бесконечностями оказался другой ноль, удаляем его
                            if (it!=nulls.end()){
                                if (nulls.begin()+k>it){k--;}
                                nulls.erase(it);

                            }
                        }
                        matrix[size*n+j]=INFINITY; //всю строку кроме нуля делаем бесконечностью


                    }
                }
                for (int j=0;j<size;j++){
                    if (j!=n){
                        if (matrix[size*j+m]==0){
                            auto it=std::find(nulls.begin(),nulls.end(),std::make_pair(j,m)); //если в столбце с бесконечностями оказался другой ноль, удаляем его
                            if (it!=nulls.end()){
                                if (nulls.begin()+k>it){k--;}
                                nulls.erase(it);
                            }
                        }
                        matrix[size*j+m]=INFINITY; //весь столбец делаем бесконечностями, кроме самого нуля
                    }
                }


                if (matrix[size*m+n]==0){
                    auto it=std::find(nulls.begin(),nulls.end(),std::make_pair(m, n));
                    if (it!=nulls.end()){
                        if (nulls.begin()+k>it){k--;}
                        nulls.erase(it); //если был 0 на месте, где мы поставили inf, убрать из списка нулей
                    }
                }
                matrix[size*m+n]=INFINITY;//чтобы не было колец из 2 городов
                unfinished--;
                route[n]=m;//ещё одно ребро в маршруте
                if (start[n]==-1 && start[m]==-1){
                    start[n]=n;
                    start[m]=n;
                    end[n]=m;
                    end[m]=m;

                }else if (start[m]==-1){
                    start[m]=start[n];
                    end[m]=m;
                    for (int j=0;j<size;j++){
                        if (end[j]==n){end[j]=m;}
                    }
                }else if (end[n]==-1){
                    start[n]=n;
                    end[n]=end[m];
                    for (int j=0;j<size;j++){
                        if (start[j]==m){start[j]=n;}
                    }
                }else{
                    for (int j=0;j<size;j++){
                        if (end[j]==n){end[j]=end[m];}
                    }
                    for (int j=0;j<size;j++){
                        if (start[j]==m){start[j]=start[n];}
                    }  //изменили параметры цепочек, в которые входят элементы
                }
                nulls.erase(nulls.begin()+k);
            }else{
                sortedNulls.push_back(std::make_pair(std::make_pair(nulls[k].first,nulls[k].second),std::make_pair(minstr,minrow)));
                k++;
            }
            if (unfinished==2){return;}

        }
        std::sort(sortedNulls.begin(), sortedNulls.end(),
                [](const std::pair<std::pair<int,int>,std::pair<float,float>>& a, const std::pair<std::pair<int,int>,std::pair<float,float>>& b) {
                            //функция сортировки вектора с информацией о нулях матрицы
            float sumA = a.second.first + a.second.second;
            float sumB = b.second.first + b.second.second;
            return sumA > sumB;  // по убыванию
        }); //теперь вектор sortedNulls из информации о нулях отсортирован по убыванию оценки




        int i = 0;


        while (i < sortedNulls.size()) {
            float sum = sortedNulls[i].second.first + sortedNulls[i].second.second;
            int from = sortedNulls[i].first.first;
            int to = sortedNulls[i].first.second;


            if (unfinished!=1 && to==0){
                i++;
                continue;
            }
            // Проверка 2: если это последнее ребро (unfinished == 1) И замыкание цикла
            if (unfinished == 1 && to == 0 && std::all_of(start.begin(), start.end(), //я бы сказал если незаконченных городов не один и при этом пункт назначения - нулевой город,
                                                          [](int x) { return x == 0; })) {// то ребро НЕДОПУСТИМО. Иначе, то есть если или незаконченных городов
                // Этот ноль допустим (особый случай)
                break;
            }

            // Проверка 3: обычный случай - не должно быть самопересечения
            if (end[to] == from && start[from] == to) {
                i++;  // самопересечение, пропускаем
                continue;
            }

            // если дошли сюда - ноль допустим
            break;
        }


        nmax=sortedNulls[i].first.first;
        mmax=sortedNulls[i].first.second;
        int newestimation=estimation+sortedNulls[i].second.first+sortedNulls[i].second.second; // estimation для ветки, не содержащей узел с максимальной оценкой
        float* newMatrix1 = new float[size * size];
        std::copy(matrix,matrix+size*size,newMatrix1);


        //size остаётся прежним
        std::vector<std::pair<int,int>> newnulls;
        std::copy(nulls.begin(), nulls.end(), std::back_inserter(newnulls));
        if (std::find(newnulls.begin(),newnulls.end(),std::make_pair(nmax, mmax))!=newnulls.end()){
            newnulls.erase(std::find(newnulls.begin(),newnulls.end(),std::make_pair(nmax, mmax))); //список нулей обновили ЗДЕСЬ ВЫДАЁТ ОШИБКУ, ПЕРЕСМОТРЕТЬ ФОРМИРОВАНИЕ НУЛЕЙ
        }
        newMatrix1[nmax*size+mmax]=INFINITY; // отредактировали матрицу для ветки, не содержащей узел с максимальной оценкой
        for (int j=0;j<size;j++){
            if (newMatrix1[j*size+mmax]==sortedNulls[i].second.second && j!=nmax && std::find(newnulls.begin(),newnulls.end(),std::make_pair(j, mmax))==newnulls.end()){
                newnulls.push_back(std::make_pair(j,mmax));
            }
            newMatrix1[j*size+mmax]-=sortedNulls[i].second.second;
        }
        for (int j=0;j<size;j++){
            if (newMatrix1[nmax*size+j]==sortedNulls[i].second.first &&j!=mmax && std::find(newnulls.begin(),newnulls.end(),std::make_pair(nmax, j))==newnulls.end()){
                newnulls.push_back(std::make_pair(nmax,j));
            }
            newMatrix1[nmax*size+j]-=sortedNulls[i].second.first;
        }
        //route,start,unfinished и end остаются неизменными
        SubTask subtask1=SubTask(newMatrix1,unfinished,route,end,start,newestimation,newnulls,size); //подзадача, не содержащая максимального ребра
        //КОНСТРУИРОВАНИЕ ПОДЗАДАЧИ, СОДЕРЖАЩЕЙ МАКСИМАЛЬНОЕ РЕБРО
        auto zero=std::find(nulls.begin(),nulls.end(),std::make_pair(nmax,mmax));
        if (zero!=nulls.end()){
            nulls.erase(zero);
        }
        for (int j=0;j<size;j++){
            if (j!=mmax){
                if (matrix[size*nmax+j]==0){
                    auto it=std::find(nulls.begin(),nulls.end(),std::make_pair(nmax,j)); //если в строке с бесконечностями оказался другой ноль, удаляем его
                    if (it!=nulls.end()){
                        nulls.erase(it);
                    }
                }
                matrix[size*nmax+j]=INFINITY; //всю строку кроме нуля делаем бесконечностью


            }
        }
        for (int j=0;j<size;j++){
            if (j!=nmax){
                if (matrix[size*j+mmax]==0){
                    auto it=std::find(nulls.begin(),nulls.end(),std::make_pair(j,mmax)); //если в столбце с бесконечностями оказался другой ноль, удаляем его
                    if (it!=nulls.end()){
                        nulls.erase(it);
                    }
                }
                matrix[size*j+mmax]=INFINITY; //весь столбец делаем бесконечностями, кроме самого нуля
            }
        }


        if (matrix[size*mmax+nmax]==0){
            auto it=std::find(nulls.begin(),nulls.end(),std::make_pair(mmax, nmax));
            if (it!=nulls.end()){
                nulls.erase(it); //если был 0 на месте, где мы поставили inf, убрать из списка нулей
            }
        }
        matrix[size*mmax+nmax]=INFINITY;//чтобы не было колец из 2 городов

        route[nmax]=mmax;//ещё одно ребро в маршруте
        //столбец и строку, в которых находится элемент с максимальной оценкой, заменили на бесконечность, т.е для ребра a->b кроме как в b никуда нельзя и в b ниоткуда нельзя
        estimation=this->MatrixReduct()+estimation;// оценка снизу для нового узла, матрицу тоже изменили. size тот же
        if (estimation==INFINITY){
            subtasks.erase(subtasks.begin());
            return;
        }
        //newUnfinished=node.unfinished-1
        unfinished--;

        if (start[nmax]==-1 && start[mmax]==-1){
            start[nmax]=nmax;
            start[mmax]=nmax;
            end[nmax]=mmax;
            end[mmax]=mmax;

        }else if (start[mmax]==-1){
            start[mmax]=start[nmax];
            end[mmax]=mmax;
            for (int k=0;k<size;k++){
                if (end[k]==nmax){end[k]=mmax;}
            }
        }else if (end[nmax]==-1){
            start[nmax]=nmax;
            end[nmax]=end[mmax];
            for (int k=0;k<size;k++){
                if (start[k]==mmax){start[k]=nmax;}
            }
        }else{
            for (int k=0;k<size;k++){
                if (end[k]==nmax){end[k]=end[mmax];}
            }
            for (int k=0;k<size;k++){
                if (start[k]==mmax){start[k]=start[nmax];}
            }  //изменили параметры цепочек, в которые входят элементы
        }

        i=0;
        while (i<nulls.size()){
            if (nulls[i].first==nmax || nulls[i].second==mmax){
                nulls.erase(nulls.begin()+i); //удалили все нули которые были в строке или столбце с максимальным нулём
            }else{i++;}
        }
        subtasks.push_back(std::move(subtask1));
    }
};


std::vector<int> LittleAlg(float* matrix, int size){
    std::cout << "1. Начало LittleAlg, size=" << size << "\n";
    std::vector<SubTask> subtasks;
    std::cout << "2. Вектор создан, size=" << subtasks.size() << ", capacity=" << subtasks.capacity() << "\n";

    subtasks.reserve(size);
    std::cout << "3. После reserve, capacity=" << subtasks.capacity() << "\n";

    float* cpymatrix = new float[size*size];
    std::copy(matrix, matrix+size*size, cpymatrix);
    std::cout << "4. Матрица скопирована\n";

    SubTask starttask = SubTask(cpymatrix,size);
    std::cout << "5. starttask создан\n";

    subtasks.push_back(std::move(starttask));
    std::cout << "6. starttask перемещен в вектор, размер вектора=" << subtasks.size() << "\n";
    float bestLength=INFINITY;
    std::vector<int> way(size);


    while (subtasks.size()!=0){
        if (subtasks.empty()) break;
        if (subtasks[0].unfinished==2){




            std::vector<int> unfinished_cities;
            for (int i=0;i<size;i++){
                if (subtasks[0].route[i]==-1){
                unfinished_cities.push_back(i);}
            }

            int isolated_city=-1;
            for (int i=0;i<2;++i){
                if (subtasks[0].start[unfinished_cities[i]]==-1){
                    subtasks[0].route[unfinished_cities[i]]=subtasks[0].start[unfinished_cities[1-i]];
                    subtasks[0].route[unfinished_cities[1-i]]=unfinished_cities[i];
                    isolated_city=1;//изолированный город
                }
            }
            if (isolated_city==-1){
                subtasks[0].route[unfinished_cities[0]]=subtasks[0].start[unfinished_cities[1]];
                subtasks[0].route[unfinished_cities[1]]=subtasks[0].start[unfinished_cities[0]];
                subtasks[0].estimation+=subtasks[0].matrix[size*unfinished_cities[0]+subtasks[0].start[unfinished_cities[1]]]+subtasks[0].matrix[size*unfinished_cities[1]+subtasks[0].start[unfinished_cities[0]]];
            }



            if (subtasks[0].estimation<bestLength){
                bestLength=subtasks[0].estimation;
                int n=0;
                for (int i=0;i<size;i++){
                    way[i]=subtasks[0].route[n];
                    n=subtasks[0].route[n];
                }
            }





            int i=0;
            while (i<subtasks.size()){
                if (subtasks[i].estimation>=bestLength){
                    subtasks.erase(subtasks.begin()+i);
                }else{
                    i++;
                }
            }// почистили subtasks, после того, как получили новый путь


        }else{
            subtasks[0].GetSubTasks(subtasks);
             ShowMatrix(subtasks[0].matrix,size);
            std::cout<<'\n';
        }
    }
    return way;
}

