#include "base/link.h"

using namespace bmpf;

/**
 * Получить строковое представление звена
 * @return Строковое представление звена
 */
std::string Link::toString() const {
    return "model_path=" + modelPath + "\nname=" + name;
}