#include <fstream>
#include <utility>
#include "path_optimizing_client.h"

/**
 * Инициализация клиента
 * @param port порт
 * @param addr адрес сервера
 * @param path путь, который нужно оптимизировать
 * @param delay  пауза в микросекундах между итерациями цикла
 */
void PathOptimizingClient::init(
        int port, const char *addr, std::vector<std::vector<double>> path, int delay
) {
    BaseClient::init(port, addr, delay);
    _path = std::move(path);
}

/**
 * главный цикл оптимизации
 */
void PathOptimizingClient::mainLoop() {

    requestReady();
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    startOptimizePath();
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    getResult();

    std::cout << "SAVED TO FILE" << std::endl;

}

/**
 * Запрос, готов ли оптимизатор
 */
void PathOptimizingClient::requestReady() {
    bool isReady = false;

    while (!isReady) {
        Json::Value json;
        json["command"] = COMMAND_IS_READY;
        json["data"] = {};

        std::string jsonText = "*" + json.toStyledString() + "*";

        send(clientFD, jsonText.c_str(), jsonText.size(), 0);

        char buffer[1024] = {0};

        int valread = read(clientFD, buffer, 1024);

        Json::Reader reader;
        Json::Value obj;
        reader.parse(buffer, obj); // reader can also read strings

        if (obj["command"].asInt() == COMMAND_IS_READY) {
            isReady = obj["data"].asBool();
            std::cout << obj["data"] << std::endl;
            std::cout << isReady << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

}

/**
 * Запуск оптимизации
 */
void PathOptimizingClient::startOptimizePath() {
    int valread;
    char buffer[1024] = {0};
    Json::Value json;
    json["command"] = COMMAND_START_OPTIMIZE_PATH;
    Json::Value jsonData;
    for (int i = 0; i < _path.size(); i++) {
        Json::Value jsonState;
        for (int j = 0; j < _path.at(i).size(); j++) {
            jsonState[j] = _path.at(i).at(j);
        }
        jsonData[i] = jsonState;
    }
    json["data"] = jsonData;

    std::cout << json.toStyledString() << std::endl;

    std::string jsonText = "*" + json.toStyledString() + "*";
    send(clientFD, jsonText.c_str(), jsonText.size(), 0);
    valread = read(clientFD, buffer, 1024);
}

/**
 * Запрос результата
 */
void PathOptimizingClient::getResult() {

    int status, valread;

    char buffer[1024] = {0};
    bool isReady = false;
    while (!isReady) {
        Json::Value json;
        json["command"] = COMMAND_OPTIMIZE_PATH;
        json["data"] = {};

        std::string jsonText = "*" + json.toStyledString() + "*";
        send(clientFD, jsonText.c_str(), jsonText.size(), 0);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        valread = read(clientFD, buffer, 1024);

        for (int i = 0; i < valread; i++) {
            if (buffer[i] == '*') {
                if (!_commandBuf.empty()) {
                    Json::Reader reader;
                    Json::Value obj;
                    reader.parse(_commandBuf, obj); // reader can also read strings

                    if (obj["data"].isNull()) {
                        continue;
                    }

                    std::ofstream myfile;
                    myfile.open(_savePath);
                    myfile << obj["data"].toStyledString();
                    myfile.close();
                    isReady = true;
                }
                _commandBuf.clear();
            } else {
                _commandBuf += buffer[i];
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(_delay));
    }

}