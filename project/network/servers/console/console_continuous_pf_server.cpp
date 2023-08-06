#include "base_server.h"
#include "continuous_pf_server.h"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#define MAIN_LOOP_DELAY_MCS 10
#define DEFAULT_PORT_VALUE 8080
#define DEFAULT_MAX_CLIENTS  10
#define THREAD_CNT 1
#define DEFAULT_MAX_OPEN_SET_SIZE  1000
#define DEFAULT_GRID_SIZE  10
#define DEFAULT_MAX_NODE_CNT  3000
#define DEFAULT_CHECK_CNT 10

int PORT;
int MAX_CLIENTS;
bool SHOW_TRACE;
int MAX_OPEN_SET_SIZE;
int GRID_SIZE;
int MAX_NODE_CNT;
int CHECK_CNT;
std::string scenePath;


bool init(int argc, char const **argv) {
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("port,p", po::value<int>(), "port value")
            ("scene,s", po::value<std::string>(), "scene path")
            ("maxClients,c", po::value<int>(), "max count of clients")
            ("trace,t", po::value<bool>(), "show trace")
            ("maxOpenSetSize,o", po::value<std::string>(), "max size of open set")
            ("gridSize,g", po::value<std::string>(), "planning grid size")
            ("maxNodeCnt,n", po::value<std::string>(), "max size of closed set")
            ("checkCnt,e", po::value<int>(), "count path stage checks");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return false;
    }

    if (vm.count("scene")) {
        scenePath = vm["scene"].as<std::string>();
    } else {
        std::cout << "Scene was not set." << std::endl;
        return false;
    }

    if (vm.count("port")) {
        PORT = vm["port"].as<int>();
    } else {
        PORT = DEFAULT_PORT_VALUE;
    }

    if (vm.count("maxClients")) {
        MAX_CLIENTS = vm["maxClients"].as<int>();
    } else {
        MAX_CLIENTS = DEFAULT_MAX_CLIENTS;
    }

    if (vm.count("trace")) {
        SHOW_TRACE = vm["trace"].as<bool>();
    } else {
        SHOW_TRACE = false;
    }

    if (vm.count("maxOpenSetSize")) {
        MAX_OPEN_SET_SIZE = vm["maxOpenSetSize"].as<int>();
    } else {
        MAX_OPEN_SET_SIZE = DEFAULT_MAX_OPEN_SET_SIZE;
    }

    if (vm.count("gridSize")) {
        GRID_SIZE = vm["gridSize"].as<int>();
    } else {
        GRID_SIZE = DEFAULT_GRID_SIZE;
    }

    if (vm.count("maxNodeCnt")) {
        MAX_NODE_CNT = vm["maxNodeCnt"].as<int>();
    } else {
        MAX_NODE_CNT = DEFAULT_MAX_NODE_CNT;
    }

    if (vm.count("checkCnt")) {
        CHECK_CNT = vm["checkCnt"].as<int>();
    } else {
        CHECK_CNT = DEFAULT_CHECK_CNT;
    }


    return true;
}


int main(int argc, char const *argv[]) {

    if (!init(argc, argv))
        return -1;

    // без вызова этого метода отваливается север после отключения клиента
    BaseServer::initLinuxServerSocket();

    // загружаем сцену
    std::shared_ptr<bmpf::Scene> sceneWrapper = std::make_shared<bmpf::Scene>();
    sceneWrapper->loadFromFile(scenePath);


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
