#include "base_server.h"

/**
 * без вызова этого метода отваливается север после отключения клиента
 */
void BaseServer::initLinuxServerSocket() {
    // без этих четырёх строк вылетает сервер
    sigset_t set;
    sigaddset(&set, SIGPIPE);
    int retcode = sigprocmask(SIG_BLOCK, &set, nullptr);
    if (retcode == -1) bmpf::errMsg("sigprocmask");
}

/**
 * главный цикл обработки запросов
 */
void BaseServer::mainLoop() {
    int new_socket;
    _terminated = false;
    while (!_terminated) {
        if ((new_socket = acceptNewSocket()) < 0) {
            perror("accept");
            exit(EXIT_FAILURE);
        }
        onSocketConnected(new_socket);
        processRequests(new_socket);
        std::this_thread::sleep_for(std::chrono::microseconds(_delay));
    }
}

/**
 * Инициализация сервера
 * @param port порт
 * @param maxClients максимальное число клиентов
 * @param delay  пауза в микросекундах между итерациями цикла
 */
void BaseServer::init(int port, int maxClients, int delay) {
    _delay = delay;
    _port = port;

    sem_init(&_sem, 0, 1);
    _addrLen = sizeof(_address);

    // Создаём файловый дескриптор
    if ((_serverFD = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        bmpf::errMsg("socket failed");
        exit(EXIT_FAILURE);
    }

    int opt = 1;

    // настраиваем сокет
    if (setsockopt(_serverFD, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    _address.sin_family = AF_INET;
    _address.sin_addr.s_addr = INADDR_ANY;
    _address.sin_port = htons(_port);

    // привязываем сокет к серверу
    if (bind(_serverFD, (struct sockaddr *) &_address, sizeof(_address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    // ожидание клиентов
    if (listen(_serverFD, maxClients) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
}

/**
 * Обработка запроса клиента
 * @param clientSocket fd клиента
 */
void BaseServer::processRequests(int clientSocket) {
    while (true) {
        char buffer[1024] = {0};
        int valread = read(clientSocket, buffer, 1024);
        if (valread <= 0) {
            close(clientSocket);
            return;
        }
        for (int i = 0; i < valread; i++) {
            if (buffer[i] == '*') {
                if (!_commandBuf.empty()) {
                    Json::Reader reader;
                    Json::Value obj;
                    reader.parse(_commandBuf, obj);

                    int curCommand = obj["command"].asInt();
                    processCommand(clientSocket, curCommand, obj["data"]);
                }
                _commandBuf.clear();
            } else {
                _commandBuf += buffer[i];
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(_delay));
    }
}
