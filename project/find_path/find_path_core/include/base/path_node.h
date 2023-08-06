#pragma once

#include <memory>
#include <utility>
#include <vector>

namespace bmpf {
    /**
     * Нода планирощика
     */
    struct PathNode {
        /**
         * Конструктор
         * @param coords координаты
         * @param parent предок
         * @param sum значение
         */
        PathNode(std::vector<int> coords, const std::shared_ptr<PathNode> &parent, double sum)
                : coords(std::move(coords)), parent(parent), sum(sum) {

            if (!parent)
                order = 0;
            else
                order = parent->order + 1;
        }

        /**
         * Строковое представление ноды
         * @return строковое представление ноды
         */
        std::string toString() const {
            std::string result = "{";
            char buf[256];
            sprintf(buf, "sum:%.3f", sum);
            result += buf;
            return result + "}";
        }

        /**
         * Координаты
         */
        std::vector<int> coords;
        /**
         * Указатель на ноду предка
         */
        std::shared_ptr<PathNode> parent;
        /**
         * метрика расстояния от стартовой точки до рассматриваемой
         */
        double sum;
        /**
         * сколько нод отделяют рассматриваемую от стартовой
         */
        unsigned int order;
    };

    /**
     * Указатель на ноду с возможностью сравнивания
     */
    struct PathNodePtr {
        /**
         * Конструктор
         * @param ptr указатель
         */
        explicit PathNodePtr(std::shared_ptr<PathNode> ptr) : ptr(std::move(ptr)) {}

        std::shared_ptr<PathNode> ptr;

        bool operator<(const PathNodePtr &obj) const {
            return (this->ptr->sum < obj.ptr->sum);
        }
    };
}