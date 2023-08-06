#include <fstream>
#include <utility>
#include "trajectory_finding_client.h"

/**
 * Инициализация клиента
 * @param port порт
 * @param addr адрес сервера
 * @param delay  пауза в микросекундах между итерациями цикла
 * @param start стартовое состояние
 * @param end конечное состояние
*/
void TrajectoryFindingClient::init(
        int port, const char *addr, std::vector<double> start, std::vector<double> end, int delay
) {
    BaseClient::init(port, addr, delay);
    _start = std::move(start);
    _end = std::move(end);
}


/**
 * главный цикл планирования
 */
void TrajectoryFindingClient::mainLoop() {

    requestReady();
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    startFindTrajectory();
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    getResult();

    std::cout << "SAVED TO FILE" << std::endl;

}

/**
 * Запрос, готов ли планировщик
 */
void TrajectoryFindingClient::requestReady() {
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
        reader.parse(buffer, obj);

        if (obj["command"].asInt() == COMMAND_IS_READY) {
            isReady = obj["data"].asBool();
            std::cout << obj["data"] << std::endl;
            std::cout << isReady << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(_delay));
    }

}

/**
 * Запуск планирования траектории
 */
void TrajectoryFindingClient::startFindTrajectory() {
    int valread;
    char buffer[1024] = {0};
    Json::Value json;
    json["command"] = COMMAND_START_FIND_TRAJECTORY;
    for (int i = 0; i < _start.size(); i++)
        json["data"]["start"][i] = _start.at(i);
    for (int i = 0; i < _end.size(); i++)
        json["data"]["end"][i] = _end.at(i);

    std::string jsonText = "*" + json.toStyledString() + "*";

    send(clientFD, jsonText.c_str(), jsonText.size(), 0);
    valread = read(clientFD, buffer, 1024);
}

/**
 * запрос результата планирования
 */
void TrajectoryFindingClient::getResult() {

    int valread;

    char buffer[1024] = {0};
    bool isReady = false;
    while (!isReady) {
        Json::Value json;
        json["command"] = COMMAND_GET_POSITIONS;
        json["data"] = 0.0;

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

                    isReady = true;
                }
                _commandBuf.clear();
            } else {
                _commandBuf += buffer[i];
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    _commandBuf.empty();

    std::vector<std::vector<double>> trajectory;

    Json::Value jsonTrajectory;

    std::cout << "get trajectory values" << std::endl;

    int j = 0;
    for (double i = 0; i < 0.5; i += 0.01, j++) {
        Json::Value json;
        switch (_lvl) {
            case 0:
                json["command"] = COMMAND_GET_POSITIONS;
                break;
            case 1:
                json["command"] = COMMAND_GET_SPEED;
                break;
            case 2:
                json["command"] = COMMAND_GET_ACCELERATIONS;
                break;
        }
        json["data"] = i;

        std::string jsonText = "*" + json.toStyledString() + "*";
        send(clientFD, jsonText.c_str(), jsonText.size(), 0);

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

                    jsonTrajectory[j] = obj["data"];
                }
                _commandBuf.clear();
            } else {
                _commandBuf += buffer[i];
            }
        }
    }

    std::ofstream myfile;
    myfile.open(_savePath);
    myfile << jsonTrajectory.toStyledString();
    myfile.close();
}