#include <netinet/in.h>
#include <iostream>
#include <arpa/inet.h>
#include <fstream>
#include "base_client.h"


/**
 * Инициализация клиента
 * @param port порт
 * @param addr адрес сервера
 * @param delay  пауза в микросекундах между итерациями цикла
 */
void BaseClient::init(int port, const char *addr, int delay) {
    _addr = addr;
    _delay = delay;
    _port = port;

    struct sockaddr_in serv_addr;

    // создаём сокет
    if ((clientFD = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cout << "Socket creation error" << std::endl;
        return;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    // задаём параметры сервера
    if (inet_pton(AF_INET, addr, &serv_addr.sin_addr) <= 0) {
        std::cout << "Invalid address/ Address not supported";
        return;
    }

    int status;
    if ((status = connect(clientFD, (struct sockaddr *) &serv_addr, sizeof(serv_addr))) < 0) {
        printf("\nConnection Failed \n");
        return;
    }

}

