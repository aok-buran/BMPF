#include <thread>
#include "solid_sync_collider.h"


using namespace bmpf;

/**
 * конструктор
 * @param mutexCnt количество мьютексов
 */
SolidSyncCollider::SolidSyncCollider(unsigned int mutexCnt) {
    if (mutexCnt > MAX_MUTES_CNT) {
        char buf[1024];
        sprintf(buf,
                "SolidSyncCollider::SolidSyncCollider() ERROR: \n mutexCnt is %u, but MAX_MUTES_CNT is %d",
                mutexCnt, MAX_MUTES_CNT
        );
        throw std::invalid_argument(buf);
    }
    _mutexCnt = mutexCnt;

    for (unsigned int i = 0; i < _mutexCnt; i++) {
        _colliders.push_back(std::make_shared<SolidCollider>());
        _colliderMutexes[i].unlock();
    }
}

/**
 * инициализация коллайдера
 *
 * @param groupedModelPaths вектор векторов путей к моделям. это связано с тем,
 * что сцена строится на основе набора описаний роботов
 * каждый из них содержит список путей к своим моделям
 * @param subColliders флаг, нудно ли составить виртуальные сцены для
 * каждого подмножества объектов сцены
 */
void SolidSyncCollider::init(std::vector<std::vector<std::string>> groupedModelPaths, bool subColliders) {
    for (unsigned int i = 0; i < _mutexCnt; i++) {
        _colliders.at(i)->init(groupedModelPaths, false);
        _colliderMutexes[i].unlock();
    }
}

/**
 * проверка соответствует ли состояние сцены столкновению
 * @param matrices список матриц преобразований звеньев
 * @return флаг, соответствует ли состояние сцены столкновению
 */
bool SolidSyncCollider::isCollided(std::vector<Eigen::Matrix4d> matrices) {
    // повторяем, пока не будет выполнена проверка на
    // том или ином коллайдере
    while (true) {
        // перебираем мьютексы и ищем свободный
        for (unsigned i = 0; i < _mutexCnt; i++)
            if (_colliderMutexes[i].try_lock()) {
                // если получилось его заблокировать, запускаем проверку
                // на соответствующем коллайдере
                bool result = _colliders.at(i)->isCollided(matrices);
                // освобождаем мьютекс
                _colliderMutexes[i].unlock();
                // возвращаем результат проверки
                return result;
            }
        // делаем паузу в одну микросекунду
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}

/**
 * @brief проверка соответствует ли состояние сцены столкновению
 * проверка соответствует ли состояние сцены (список матриц преобразований звеньев
 * только для задействованных объектов)
 * столкновению при этом учитываются только объекты с индексами из robotIndexes
 * @param matrices список матриц преобразований звеньев
 * @param robotIndexes индексы роботов
 * @return флаг, соответствует ли состояние сцены столкновению
 */
bool SolidSyncCollider::isCollided(std::vector<Eigen::Matrix4d> matrices, std::vector<int> robotIndexes) {
    // повторяем, пока не будет выполнена проверка на
    // том или ином коллайдере
    while (true) {
        // перебираем мьютексы и ищем свободный
        for (unsigned i = 0; i < _mutexCnt; i++)
            if (_colliderMutexes[i].try_lock()) {
                // если получилось его заблокировать, запускаем проверку
                // на соответствующем коллайдере
                bool result = _colliders.at(i)->isCollided(matrices, robotIndexes);
                // освобождаем мьютекс
                _colliderMutexes[i].unlock();
                // возвращаем результат проверки
                return result;
            }
        // делаем паузу в одну микросекунду
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}