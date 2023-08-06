#pragma  once
#include <QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include "scene_editor_widget.h"
#include "log.h"

#include <QSlider>
#include <QVBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>
#include <solid_collider.h>
#include <QtWidgets/QFileDialog>
#include <one_direction_path_finder.h>

QT_BEGIN_NAMESPACE
class QSlider;
class QPushButton;
QT_END_NAMESPACE

class SceneEditorWidget;
class SceneEditorMainWindow;

/**
 * Класс главного окна редактора сцены
 */
class SceneEditorWindow: public QWidget {
Q_OBJECT

public:
    /**
     * Конструктор
     * @param mw главное окно
     * @param scenePath путь к описанию сцены
     */
    SceneEditorWindow(SceneEditorMainWindow *mw, const std::string& scenePath);
    /**
     * Сохранить сцену
     * @param path путь к файлу
     */
    void saveScene(const std::string& path);

protected:
    /**
     * Обработчик события клавиатуры
     * @param event событие
     */
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

private slots:
    /**
     * Открепить окно
     */
    void dockUndock();
    /**
     * Добавить робота
     */
    void doAdd();
    /**
     * Удалить робота
     */
    void doDelete();
    /**
     * Следующий объект
     */
    void doNext();
    /**
     * Предыдущий объект
     */
    void doPrev();


private:

    /**
     * Создать слайдер
     * @param min минимальное значение
     * @param max максимальное значение
     * @return новый слайдер
     */
    QSlider *createSlider(int min, int max);
    /**
     * Получить слайдер с заголовком
     * @param pSlider слайдер
     * @param pLayout разметка
     * @param string заголовок
     * @return слайдер с заголовком
     */
    QWidget *getSliderWithCaption(QSlider *pSlider, QVBoxLayout *pLayout, const char string[2]);


    SceneEditorWidget *glWidget;
    QPushButton *dockBtn;
    QPushButton *addBtn;
    QPushButton *deleteBtn;

    QLabel *translateLabel;
    QLabel *rotateLabel;
    QLabel *scaleLabel;

    QSlider *translateXSlider;
    QSlider *translateYSlider;
    QSlider *translateZSlider;

    QSlider *rotateXSlider;
    QSlider *rotateYSlider;
    QSlider *rotateZSlider;

    QSlider *scaleXSlider;
    QSlider *scaleYSlider;
    QSlider *scaleZSlider;

    QPushButton * nextBtn;
    QPushButton * prevBtn;

    SceneEditorMainWindow *mainWindow;

    std::shared_ptr<bmpf::PathFinder> _pathFinder;
};
