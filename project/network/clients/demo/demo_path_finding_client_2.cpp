#include "iostream"
#include "base_client.h"
#include "path_finding_client.h"
#include <unistd.h>
#include <thread>
#include <json/value.h>


#define PORT 8080
#define MAIN_LOOP_DELAY_MCS 10

const char *addr = "127.0.0.1";

const char *savePath = "../../../../out/client/test.txt";


std::vector<double> start
        {-1.696, 0.453, -1.582, -0.569, 0.827, -2.817, -2.769, 0.360, 1.462, 1.441, -1.827, 5.589, -2.054, -1.892,
         -0.302, -1.915, 1.601, 5.947, 1.238, -0.023, -0.341, 0.757, 0.534, 0.494};
std::vector<double> end
        {0.759, -2.957, 0.393, 3.176, 0.857, -4.351, 0.192, -2.326, 0.592, -0.243, 0.344, -3.707, -0.772, -0.119,
         -1.855, -1.959, -1.745, -2.263, 1.309, -0.623, 0.860, -2.320, 1.961, 1.648};


int main(int argc, char const *argv[]) {
    // создаём сервер
    std::shared_ptr<PathFindingClient> client = std::make_shared<PathFindingClient>(savePath);

    // инициализируем его
    client->init(PORT, addr, start, end, MAIN_LOOP_DELAY_MCS);

    std::cout << "SERVER STARTED AT PORT " << PORT << std::endl;

    // запускаем цикл обработки запросов
    client->mainLoop();

    // закрываем подключение
    close(client->getClientFd());
    return 0;
}