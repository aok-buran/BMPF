#include "iostream"
#include "path_optimizing_client.h"
#include <unistd.h>
#include <thread>
#include <json/value.h>
#include <fstream>


#define PORT 8080
#define MAIN_LOOP_DELAY_MCS 10

const char *addr = "127.0.0.1";

const char *sourcePath = "../../../../out/client/test.txt";
const char *savePath = "../../../../out/client/test2.txt";


int main(int argc, char const *argv[]) {
    // создаём сервер
    std::shared_ptr<PathOptimizingClient> client = std::make_shared<PathOptimizingClient>(savePath);


    std::ifstream ifs(sourcePath, std::ios_base::binary);


    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    Json::Reader reader;
    Json::Value obj;

    reader.parse(content, obj); // reader can also read strings


    std::vector<std::vector<double>> path;


    for (Json::Value state: obj["states"]) {
        std::vector<double> curState;
        //  infoMsg(path[0]["states"]);
        for (Json::Value coord: state) {
            curState.emplace_back(coord.asDouble());
        }
        path.emplace_back(curState);
    }

    // инициализируем его
    client->init(PORT, addr, path, MAIN_LOOP_DELAY_MCS);

    std::cout << "SERVER STARTED AT PORT " << PORT << std::endl;

    // запускаем цикл обработки запросов
    client->mainLoop();

    // закрываем подключение
    close(client->getClientFd());
    return 0;
}