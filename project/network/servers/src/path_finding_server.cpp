#include "path_finding_server.h"

/**
 * Начать планирование
 * @param pfs сервер планирования
 * @param pf планировщик пути
 * @param clientSocket fd клиента
 * @param start стартовое состояние
 * @param end конечное состояние
 */
void PathFindingServer::startFindPath(PathFindingServer *pfs, std::shared_ptr<bmpf::PathFinder> pf, int clientSocket,
                                      std::vector<double> start, std::vector<double> end) {

    int errorCode;

    auto path = pf->findPath(start, end, errorCode);
    sem_wait(&(pfs->_sem));

    auto it = pfs->paths.find(clientSocket);
    if (it != pfs->paths.end())
        it->second = path;
    else
        pfs->paths.insert({clientSocket, path});

    pfs->pathFindingReady.at(clientSocket) = true;
    sem_post(&(pfs->_sem));
}

/**
 * Обработка подключения нового клиента
 * @param clientSocket fd клиента
 */
void PathFindingServer::onSocketConnected(int clientSocket) {
    auto pf = createPathFinder(clientSocket);

    auto it = pfs.find(clientSocket);
    if (it != pfs.end())
        it->second = pf;
    else
        pfs.insert({clientSocket, pf});

    auto it2 = pathFindingReady.find(clientSocket);
    if (it2 != pathFindingReady.end())
        it2->second = false;
    else
        pathFindingReady.insert({clientSocket, false});

}

/**
 * Обработка запроса клиента
 * @param clientSocket fd клиента
 */
void PathFindingServer::processCommand(int clientSocket, int command, Json::Value jsonData) {
    switch (command) {
        case COMMAND_IS_READY: {
            Json::Value json;
            json["command"] = COMMAND_IS_READY;

            if (!pfs.count(clientSocket)) {
                json["data"] = false;
            } else {
                json["data"] = pfs.at(clientSocket)->isReady();
            }
            std::string jsonText = json.toStyledString();

            send(clientSocket, jsonText.c_str(), jsonText.size(), 0);
            break;
        }
        case COMMAND_START_FIND_PATH: {
            Json::Value json2;
            json2["command"] = COMMAND_START_FIND_PATH;
            std::vector<double> start;
            for (auto &val: jsonData["start"]) {
                start.emplace_back(val.asDouble());
            }
            std::vector<double> end;
            for (auto &val: jsonData["end"]) {
                end.emplace_back(val.asDouble());
            }

            sem_wait(&_sem);
            pathFindingReady.at(clientSocket) = false;
            sem_post(&_sem);

            std::thread t1(startFindPath, this, pfs.at(clientSocket), clientSocket, start, end);
            t1.join();

            std::string jsonText2 = json2.toStyledString();
            send(clientSocket, jsonText2.c_str(), jsonText2.size(), 0);

            break;
        }
        case COMMAND_FIND_PATH_RESULT: {
            Json::Value json2;
            json2["command"] = COMMAND_FIND_PATH_RESULT;

            sem_wait(&_sem);

            bool pfReady = pathFindingReady.at(clientSocket);
            sem_post(&_sem);

            if (!pfReady) {
                json2["data"] = {};
            } else {
                bmpf::infoMsg("FOUND PATH: SIZE = ", paths.at(clientSocket).size());

                Json::Value jsonPath;

                sem_wait(&_sem);
                for (int i = 0; i < paths.at(clientSocket).size(); i++) {
                    Json::Value state;
                    for (int j = 0; j < paths.at(clientSocket).at(i).size(); j++) {
                        state[j] = paths.at(clientSocket).at(i).at(j);
                    }
                    jsonPath[i] = state;
                }

                pathFindingReady.at(clientSocket) = false;

                sem_post(&_sem);

                Json::Value json;
                json["states"] = jsonPath;
                json["scene"] = pfs.at(clientSocket)->getScene()->getScenePath();

                json2["data"] = json;
                std::string jsonText2 = "*" + json2.toStyledString() + "*";

                send(clientSocket, jsonText2.c_str(), jsonText2.length(), 0);
            }
            break;
        }
    }
}
