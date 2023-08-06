#include "render_single_path_window.h"
#include "render_single_path_main_window.h"
#include "one_direction_path_finder.h"

#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>
#include <utility>

RenderSinglePathWindow::RenderSinglePathWindow(RenderSinglePathMainWindow *mw, std::string logPath)
        : mainWindow(mw) {
    _flgPlay = false;

    timeSlider = createSlider();

    glWidget = new RenderSinglePathWidget(timeSlider, mw);

    std::string scenePath;
    auto path = bmpf::PathFinder::loadPathFromFile(logPath,scenePath);

    std::shared_ptr<bmpf::Scene> sceneWrapper = std::make_shared<bmpf::Scene>();
    sceneWrapper->loadFromFile(scenePath);

    std::shared_ptr<bmpf::PathFinder> pathreader =
            std::make_shared<bmpf::OneDirectionPathFinder>(sceneWrapper, true, 100, 10, 4000, 1, 2);

    // infoMsg("pathreader init");
    glWidget->setPathFinder(pathreader,path);

    //infoMsg("pathreader seted");

    timeSlider->setMinimum(0);
    timeSlider->setValue(0);
    timeSlider->setMaximum(path.size()* 1000);
    timeSlider->update();

    connect(timeSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setTime(int)));
    connect(glWidget, SIGNAL(timeChanged(int)), timeSlider, SLOT(setValue(int)));

    auto *mainLayout = new QVBoxLayout;
    auto *container = new QHBoxLayout;
    container->addWidget(glWidget);
    container->addWidget(timeSlider);

    QWidget *w = new QWidget;
    w->setLayout(container);
    mainLayout->addWidget(w);

    auto *btnContainer = new QHBoxLayout;
    QWidget *btnWidget = new QWidget;
    btnWidget->setLayout(btnContainer);
    mainLayout->addWidget(btnWidget);

    dockBtn = new QPushButton(tr("Undock"), this);
    connect(dockBtn, SIGNAL(clicked()), this, SLOT(dockUndock()));
    btnContainer->addWidget(dockBtn);

    playBtn = new QPushButton(tr("Play"), this);
    connect(playBtn, SIGNAL(clicked()), this, SLOT(play()));
    btnContainer->addWidget(playBtn);

    setLayout(mainLayout);

    timeSlider->setValue(0);

    setWindowTitle(tr("Path Visualization"));
}

QSlider *RenderSinglePathWindow::createSlider() {
    auto *slider = new QSlider(Qt::Vertical);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

void RenderSinglePathWindow::keyPressEvent(QKeyEvent *e) {
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

void RenderSinglePathWindow::dockUndock() {
    if (parent()) {
        setParent(nullptr);
        setAttribute(Qt::WA_DeleteOnClose);
        move(QApplication::desktop()->width() / 2 - width() / 2,
             QApplication::desktop()->height() / 2 - height() / 2);
        dockBtn->setText(tr("Dock"));
        show();
    } else {
        if (!mainWindow->centralWidget()) {
            if (mainWindow->isVisible()) {
                setAttribute(Qt::WA_DeleteOnClose, false);
                dockBtn->setText(tr("Undock"));
                mainWindow->setCentralWidget(this);
            } else {
                QMessageBox::information(nullptr, tr("Cannot dock"), tr("Main window already closed"));
            }
        } else {
            QMessageBox::information(nullptr, tr("Cannot dock"), tr("Main window already occupied"));
        }
    }
}

void RenderSinglePathWindow::play() {
    if (_flgPlay) {
        playBtn->setText("Play");
    } else {
        playBtn->setText("Stop");
    }
    playBtn->update();
    _flgPlay = !_flgPlay;
    timeSlider->setEnabled(!_flgPlay);

    glWidget->setFlgPlay(_flgPlay);
}


