#pragma  once

#include "render_path_widget.h"

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
class RenderPathWindow : public QWidget{
Q_OBJECT

public:
    RenderPathWindow(RenderPathMainWindow *mw, std::string pathFileName);

protected:
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

private slots:
    void dockUndock();
    void next();
    void prev();
    void play();

private:
    QSlider *createSlider();

    RenderPathWidget *glWidget;
    QSlider *timeSlider;

    QPushButton *dockBtn;
    QPushButton *nextBtn;
    QPushButton *prevBtn;
    QPushButton *playBtn;

    RenderPathMainWindow *mainWindow;

    bool _flgPlay;
};
