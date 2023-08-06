#include "trajectory_finding_server.h"

/**
 * Запуск планирования траектории
 * @param tps сервер планирования
 * @param tf планировщик
 * @param clientSocket fd клиента
 * @param start стартовое состояние
 * @param end конечное состояние
 */
void TrajectoryFindingServer::startFindTrajectory(
        TrajectoryFindingServer *tps, std::shared_ptr<bmpf::MonotoneTrajectoryFinder> tf,
        int clientSocket, std::vector<double> start, std::vector<double> end
) {

    int errorCode;

    tf->prepareTrajectory(start, end, errorCode);

    sem_wait(&(tps->_sem));
    tps->trajectoryFindingReady.at(clientSocket) = true;
    sem_post(&(tps->_sem));
}

/**
 * Обработка подключения нового клиента
 * @param clientSocket fd клиента
 */
void TrajectoryFindingServer::onSocketConnected(int clientSocket) {
    auto tf = std::make_shared<bmpf::MonotoneTrajectoryFinder>();


    tf->init(_scene, _intervalDuration, _showTrace, _maxOpenSetSize, _gridSize, _maxNodeCnt, _checkCnt, _threadCnt);

    auto it = tfs.find(clientSocket);
    if (it != tfs.end())
        it->second = tf;
    else
        tfs.insert({clientSocket, tf});

    auto it2 = trajectoryFindingReady.find(clientSocket);
    if (it2 != trajectoryFindingReady.end())
        it2->second = false;
    else
        trajectoryFindingReady.insert({clientSocket, false});

}

/**
 * Обработка запроса клиента
 * @param clientSocket fd клиента
 */
void TrajectoryFindingServer::processCommand(int clientSocket, int command, Json::Value jsonData) {
    switch (command) {
        case COMMAND_IS_READY: {
            Json::Value json;
            json["command"] = COMMAND_IS_READY;

            if (!tfs.count(clientSocket)) {
                json["data"] = false;
            } else {
                json["data"] = tfs.at(clientSocket)->getPF()->isReady();
            }
            std::string jsonText = json.toStyledString();

            send(clientSocket, jsonText.c_str(), jsonText.size(), 0);
            break;
        }
        case COMMAND_START_FIND_TRAJECTORY: {

            Json::Value json2;
            json2["command"] = COMMAND_START_FIND_TRAJECTORY;
            std::vector<double> start;
            for (auto &val: jsonData["start"]) {
                start.emplace_back(val.asDouble());
            }
            std::vector<double> end;
            for (auto &val: jsonData["end"]) {
                end.emplace_back(val.asDouble());
            }

            sem_wait(&_sem);
            trajectoryFindingReady.at(clientSocket) = false;
            sem_post(&_sem);

            std::thread t1(startFindTrajectory, this, tfs.at(clientSocket), clientSocket, start, end);
            t1.join();

            std::string jsonText2 = json2.toStyledString();

            send(clientSocket, jsonText2.c_str(), jsonText2.size(), 0);

            break;
        }
        case COMMAND_GET_POSITIONS:
        case COMMAND_GET_SPEED:
        case COMMAND_GET_ACCELERATIONS: {
            Json::Value json2;
            json2["command"] = command;

            sem_wait(&_sem);

            bool pfReady = trajectoryFindingReady.at(clientSocket);
            sem_post(&_sem);

            if (!pfReady) {
                std::cout << "trajectory is not ready";
                json2["data"] = {};
            } else {
                double tm = jsonData.asDouble();
                if (tm > tfs.at(clientSocket)->getWholeDuration())
                    tm = tfs.at(clientSocket)->getWholeDuration();


                std::vector<double> state;
                switch (command) {
                    case COMMAND_GET_POSITIONS:
                        state = tfs.at(clientSocket)->getTrajectoryPosition(tm);
                        break;
                    case COMMAND_GET_SPEED:
                        state = tfs.at(clientSocket)->getTrajectorySpeed(tm);
                        break;
                    case COMMAND_GET_ACCELERATIONS:
                        state = tfs.at(clientSocket)->getTrajectoryAcceleration(tm);
                        break;
                }

                Json::Value jsonState;
                for (int i = 0; i < state.size(); i++) {
                    jsonState[i] = state.at(i);
                }

                json2["data"] = jsonState;
                std::string jsonText2 = "*" + json2.toStyledString() + "*";

                send(clientSocket, jsonText2.c_str(), jsonText2.length(), 0);

            }
            break;
        }
    }
}

