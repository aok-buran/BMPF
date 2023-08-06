#include "base_server.h"
#include "continuous_pf_server.h"


#define PORT 8080
#define MAX_CLIENTS 10
#define MAIN_LOOP_DELAY_MCS 10
#define SHOW_TRACE false
#define MAX_OPEN_SET_SIZE 1000
#define GRID_SIZE 10
#define MAX_NODE_CNT 3000
#define CHECK_CNT 100
#define THREAD_CNT 1


const char *SCENE_PATH = "../../../../config/murdf/4robots.json";


int main(int argc, char const *argv[]) {


    // без вызова этого метода отваливается север после отключения клиента
    BaseServer::initLinuxServerSocket();

    // загружаем сцену
    std::shared_ptr<bmpf::Scene> sceneWrapper = std::make_shared<bmpf::Scene>();
    sceneWrapper->loadFromFile(SCENE_PATH);


    // создаём сервер
    std::shared_ptr<BaseServer> server = std::make_shared<ContinuousPFServer>(
            sceneWrapper, SHOW_TRACE, MAX_OPEN_SET_SIZE, GRID_SIZE, MAX_NODE_CNT, CHECK_CNT, THREAD_CNT
    );

    // инициализируем его
    server->init(PORT, MAX_CLIENTS, MAIN_LOOP_DELAY_MCS);

    bmpf::infoMsg("SERVER STARTED AT PORT ", PORT);

    // запускаем цикл обработки запросов
    server->mainLoop();

}
