#pragma once

#include <iostream>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <thread>

namespace bmpf {
    /**
     * Класс-аналог защёлки CountDownLatch из Java.
     * Пример использования закомментирован внизу файла
     */
    struct CountDownLatch {
        volatile int _rest;
        std::mutex _mtx;
        std::condition_variable _cv;

        explicit CountDownLatch(int n) : _rest(n) {}

        void await() {
            std::unique_lock<std::mutex> lck(_mtx);
            while (_rest > 0) {
                _cv.wait(lck);
                //   std::cout << "Wake up " << std::this_thread::get_id() << std::endl;
            }
        }

        void countDown() {
            std::unique_lock<std::mutex> lck(_mtx);
            --_rest;
            // std::cout << "Notify " << std::this_thread::get_id() << std::endl;
            _cv.notify_all();
        }

    };

//void exec(CountDownLatch *a) {
//    std::this_thread::sleep_for(std::chrono::seconds(2));
//    a->countDown();
//}
//int main()
//{
//
//    CountDownLatch a(3);
//    std:: thread threads[3];
//    // spawn 3 threads:
//    for (int i=0; i<3; ++i)
//        threads[i] = std::thread(exec,&a);
//
//    std::cout << "3 threads ready to race...\n";
//    a.await();
//    std::cout << "Came to finish\n";
//
//    for (int i=0; i<3; ++i)
//        threads[i].join();
//
//    return 0;
//}

}