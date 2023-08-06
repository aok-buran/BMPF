#include "iostream"
#include "trajectory_finding_client.h"
#include <unistd.h>
#include <thread>
#include <json/value.h>


#define PORT 8080
#define MAIN_LOOP_DELAY_MCS 10

const char *addr = "127.0.0.1";

const char *savePath = "../../../../out/client/test4.txt";

std::vector<double> start
        {-2.372, -2.251, 1.977, 0.031, 1.885, 5.093, -2.043, -0.717, -0.893, 0.307, 0.687, -0.148, 0.723, 0.667,
         -1.421,
         -2.498, 1.934, -4.705, -2.144, -2.477, 1.529, 0.919, 1.333, 2.003};
std::vector<double> end
        {0.262, -3.238, 1.314, 2.603, -0.827, -3.604, -1.641, -0.440, 1.958, 1.606, 1.474, -4.645, -2.421, -0.583,
         0.134, -0.834, 2.049, -4.375, -2.353, -2.529, 0.148, -0.707, 0.145, -2.702};


int main(int argc, char const *argv[]) {
    // создаём сервер
    std::shared_ptr<TrajectoryFindingClient> client = std::make_shared<TrajectoryFindingClient>(
            savePath, 1
    );

    // инициализируем его
    client->init(PORT, addr, start, end, MAIN_LOOP_DELAY_MCS);

    std::cout << "SERVER STARTED AT PORT " << PORT << std::endl;

    // запускаем цикл обработки запросов
    client->mainLoop();

    // закрываем подключение
    close(client->getClientFd());
    return 0;
}