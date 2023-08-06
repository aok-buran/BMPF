#include "path_optimising_server.h"

#include <utility>

/**
 * Запуск оптимизации пути
 * @param pos сервер оптимизации
 * @param po оптимизатор пути
 * @param clientSocket fd клиента
 * @param path путь, который нужно потимизировать
 */
void PathOptimisingServer::startOptimisePath(
        PathOptimisingServer *pos, std::shared_ptr<bmpf::MedianPathOptimizer> po, int clientSocket,
        std::vector<std::vector<double>> path
) {

    auto optimisedPath = po->optimizePath(std::move(path));
    sem_wait(&(pos->_sem));

    auto it = pos->paths.find(clientSocket);
    if (it != pos->paths.end())
        it->second = optimisedPath;
    else
        pos->paths.insert({clientSocket, optimisedPath});

    pos->pathOptimisingReady.at(clientSocket) = true;
    sem_post(&(pos->_sem));
}

/**
 * Обработка подключения нового клиента
 * @param clientSocket fd клиента
 */
void PathOptimisingServer::onSocketConnected(int clientSocket) {
    std::shared_ptr<bmpf::PathFinder> pf = std::make_shared<ContinuousPathFinder>(
            _scene, _showTrace, _maxOpenSetSize, _gridSize,
            _maxNodeCnt, _checkCnt, _threadCnt
    );
    auto po = std::make_shared<bmpf::MedianPathOptimizer>(
            pf, "median", _checkCnt, _filterDivideCnt, _optimizeLoopCnt
    );

    auto it = pfs.find(clientSocket);
    if (it != pfs.end())
        it->second = po;
    else
        pfs.insert({clientSocket, po});

    auto it2 = pathOptimisingReady.find(clientSocket);
    if (it2 != pathOptimisingReady.end())
        it2->second = false;
    else
        pathOptimisingReady.insert({clientSocket, false});

}

/**
 * Обработка запроса клиента
 * @param clientSocket fd клиента
 */
void PathOptimisingServer::processCommand(int clientSocket, int command, Json::Value jsonData) {

    switch (command) {
        case COMMAND_IS_READY: {
            // infoMsg("COMMAND READY");
            Json::Value json;
            json["command"] = COMMAND_IS_READY;

            if (!pfs.count(clientSocket)) {
                json["data"] = false;
            } else {
                json["data"] = pfs.at(clientSocket)->getPathFinder()->isReady();
            }
            std::string jsonText = json.toStyledString();

            send(clientSocket, jsonText.c_str(), jsonText.size(), 0);
            break;
        }
        case COMMAND_START_OPTIMIZE_PATH: {
            Json::Value json2;
            json2["command"] = COMMAND_START_OPTIMIZE_PATH;


            std::vector<std::vector<double>> path;
            for (Json::Value state: jsonData) {
                std::vector<double> stateVec;
                for (const Json::Value &coord: state) {
                    stateVec.emplace_back(coord.asDouble());
                }
                path.emplace_back(stateVec);
            }

            sem_wait(&_sem);
            pathOptimisingReady.at(clientSocket) = false;
            sem_post(&_sem);

            std::thread t1(startOptimisePath, this, pfs.at(clientSocket), clientSocket, path);
            t1.join();

            std::string jsonText2 = json2.toStyledString();

            send(clientSocket, jsonText2.c_str(), jsonText2.size(), 0);

            break;
        }
        case COMMAND_OPTIMIZE_PATH: {
            Json::Value json2;
            json2["command"] = COMMAND_OPTIMIZE_PATH;

            sem_wait(&_sem);

            bool pfReady = pathOptimisingReady.at(clientSocket);
            sem_post(&_sem);

            if (!pfReady) {
                json2["data"] = {};
            } else {
                bmpf::infoMsg("OPTIMIZE PATH: SIZE = ", paths.at(clientSocket).size());

                Json::Value jsonPath;

                sem_wait(&_sem);
                for (int i = 0; i < paths.at(clientSocket).size(); i++) {
                    Json::Value state;
                    for (int j = 0; j < paths.at(clientSocket).at(i).size(); j++) {
                        state[j] = paths.at(clientSocket).at(i).at(j);
                    }
                    jsonPath[i] = state;
                }

                pathOptimisingReady.at(clientSocket) = false;

                sem_post(&_sem);
                Json::Value json;
                json["states"] = jsonPath;
                json["scene"] = pfs.at(clientSocket)->getPathFinder()->getScene()->getScenePath();

                json2["data"] = json;

                std::string jsonText2 = "*" + json2.toStyledString() + "*";

                send(clientSocket, jsonText2.c_str(), jsonText2.length(), 0);

            }
            break;
        }
    }
}
