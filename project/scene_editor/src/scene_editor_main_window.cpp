#include "scene_editor_main_window.h"
#include "log.h"

#include <QMenuBar>
#include <QMessageBox>
#include <QtWidgets/QDesktopWidget>
#include <QtWidgets/QFileDialog>


SceneEditorMainWindow::SceneEditorMainWindow(){
    auto *menuBar = new QMenuBar;
    QMenu *menuWindow = menuBar->addMenu(tr("&Scene"));

    auto *newScene = new QAction(menuWindow);
    newScene->setText(tr("NewScene"));
    menuWindow->addAction(newScene);
    connect(newScene, SIGNAL(triggered()), this, SLOT(onNewScene()));

    auto *openScene = new QAction(menuWindow);
    openScene->setText(tr("OpenScene"));
    menuWindow->addAction(openScene);
    connect(openScene, SIGNAL(triggered()), this, SLOT(onOpenScene()));

    auto *saveScene = new QAction(menuWindow);
    saveScene->setText(tr("SaveScene"));
    menuWindow->addAction(saveScene);
    connect(saveScene, SIGNAL(triggered()), this, SLOT(onSaveScene()));

    setMenuBar(menuBar);

    QDesktopWidget dw;
    setFixedSize(1400, 800);

}

void SceneEditorMainWindow::onNewScene(){
    if (!centralWidget()) {
        _window = new SceneEditorWindow(this, "");
        setCentralWidget(_window);
    }
    else
        QMessageBox::information(nullptr, tr("Cannot add new window"), tr("Already occupied. Undock first."));
}

void SceneEditorMainWindow::onOpenScene(){
    if (!centralWidget()) {
        QString fileName = QFileDialog::getOpenFileName(this,
                                                        QString::fromUtf8("Open scene"),
                                                        "");
        if (fileName.toStdString().empty())
            return;

        _window = new SceneEditorWindow(this, fileName.toStdString());
        setCentralWidget(_window);
    }
    else
        QMessageBox::information(nullptr, tr("Cannot add new window"), tr("Already occupied. Undock first."));

}

void SceneEditorMainWindow::onSaveScene(){
    if (!centralWidget()) {
        QMessageBox::information(nullptr, tr("Error"),
                                 tr("Nothing to save. You need firstly open or create scene"));
    }
    else {
        QString fileName = QFileDialog::getSaveFileName(this,
                                                        QString::fromUtf8("Save scene"),
                                                        "");
        if (fileName.toStdString().empty())
            return;

        _window->saveScene(fileName.toStdString());
    }
}

