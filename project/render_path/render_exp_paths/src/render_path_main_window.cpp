#include "render_path_main_window.h"
#include "render_path_window.h"

#include <QMenuBar>
#include <QMessageBox>
#include <QtWidgets/QDesktopWidget>
#include <QtWidgets/QFileDialog>

RenderPathMainWindow::RenderPathMainWindow(){

    auto *menuBar = new QMenuBar;
    QMenu *menuWindow = menuBar->addMenu(tr("&File"));
    auto *addNew = new QAction(menuWindow);
    addNew->setText(tr("OpenPath"));
    menuWindow->addAction(addNew);
    connect(addNew, SIGNAL(triggered()), this, SLOT(onAddNew()));
    setMenuBar(menuBar);

    QDesktopWidget dw;
    setFixedSize(800, 800);

}

void RenderPathMainWindow::onAddNew(){
    if (!centralWidget()) {
        QString fileName = QFileDialog::getOpenFileName(this,
                                                        QString::fromUtf8("Open path"),
                                                        "");
        if (fileName.toStdString().empty())
            return;

        setCentralWidget(new RenderPathWindow(this, fileName.toStdString()));
    }
    else
        QMessageBox::information(nullptr, tr("Can't add new window"), tr("Already occupied. Undock first."));

}