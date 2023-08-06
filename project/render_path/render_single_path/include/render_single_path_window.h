#pragma  once

#include "render_single_path_widget.h"
#include "render_single_path_main_window.h"

#include <QWidget>


QT_BEGIN_NAMESPACE
class QSlider;
class QPushButton;
QT_END_NAMESPACE

class RenderPathWidget;
class RenderPathMainWindow;

/**
 * Класс главного окна
 */
class RenderSinglePathWindow : public QWidget{
Q_OBJECT

public:
    RenderSinglePathWindow(RenderSinglePathMainWindow *mw, std::string scenePath);

protected:
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

private slots:
    void dockUndock();
    void play();

private:
    QSlider *createSlider();

    RenderSinglePathWidget *glWidget;
    QSlider *timeSlider;

    QPushButton *dockBtn;
    QPushButton *playBtn;

    RenderSinglePathMainWindow *mainWindow;

    bool _flgPlay;
};
