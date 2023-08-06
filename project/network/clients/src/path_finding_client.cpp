#include <fstream>
#include <utility>
#include "path_finding_client.h"

/**
 * Инициализация клиента
 * @param port порт
 * @param addr адрес сервера
 * @param delay  пауза в микросекундах между итерациями цикла
 * @param start стартовое состояние
 * @param end конечное состояние
*/
void PathFindingClient::init(
        int port, const char *addr, std::vector<double> start, std::vector<double> end, int delay
) {
    BaseClient::init(port, addr, delay);
    _start = std::move(start);
    _end = std::move(end);
}

/**
 * главный цикл планирования
 */
void PathFindingClient::mainLoop() {

    requestReady();
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    startFindPath();
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    getResult();

    std::cout << "SAVED TO FILE" << std::endl;

}

/**
 * Запрос, готов ли планировщик
 */
void PathFindingClient::requestReady() {
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
 * Запуск планирования
 */
void PathFindingClient::startFindPath() {
    char buffer[1024] = {0};
    Json::Value json;
    json["command"] = COMMAND_START_FIND_PATH;
    for (int i = 0; i < _start.size(); i++)
        json["data"]["start"][i] = _start.at(i);
    for (int i = 0; i < _end.size(); i++)
        json["data"]["end"][i] = _end.at(i);

    std::string jsonText = "*" + json.toStyledString() + "*";

    send(clientFD, jsonText.c_str(), jsonText.size(), 0);
    read(clientFD, buffer, 1024);

}
/**
 * Запрос результата
 */
void PathFindingClient::getResult() {
    int valread;

    char buffer[1024] = {0};
    bool isReady = false;
    while (!isReady) {
        Json::Value json;
        json["command"] = COMMAND_FIND_PATH_RESULT;
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
                    reader.parse(_commandBuf, obj);

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

        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

}